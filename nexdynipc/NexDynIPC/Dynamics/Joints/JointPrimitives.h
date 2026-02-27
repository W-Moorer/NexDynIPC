#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <cmath>
#include <vector>

namespace NexDynIPC::Dynamics::JointPrimitives {

inline Eigen::Quaterniond updateRotation(const Eigen::Quaterniond& q_ref, const Eigen::Vector3d& theta) {
    const double angle = theta.norm();
    if (angle < 1e-10) {
        return q_ref;
    }
    const Eigen::Vector3d axis = theta / angle;
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    return (dq * q_ref).normalized();
}

inline Eigen::Matrix3d crossMat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d M;
    M << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;
    return M;
}

inline void computeBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& u, Eigen::Vector3d& v) {
    const Eigen::Vector3d a = axis.normalized();
    if (std::abs(a.x()) < 0.8) {
        u = a.cross(Eigen::Vector3d::UnitX()).normalized();
    } else {
        u = a.cross(Eigen::Vector3d::UnitY()).normalized();
    }
    v = a.cross(u).normalized();
}

struct BodyState {
    int global_idx = -1;
    Eigen::Vector3d p_ref = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();
};

struct EvaluatedBody {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

inline EvaluatedBody evaluateBody(const BodyState& state, const Eigen::VectorXd& x) {
    EvaluatedBody out;
    if (state.global_idx >= 0) {
        out.p = x.segment<3>(state.global_idx);
        const Eigen::Vector3d theta = x.segment<3>(state.global_idx + 3);
        out.q = updateRotation(state.q_ref, theta);
    } else {
        out.p = state.p_ref;
        out.q = state.q_ref;
    }
    return out;
}

struct PointPrimitive {
    Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();

    Eigen::Vector3d value(const EvaluatedBody& A, const EvaluatedBody& B) const {
        const Eigen::Vector3d rA = A.q * anchorA;
        const Eigen::Vector3d rB = B.q * anchorB;
        return (A.p + rA) - (B.p + rB);
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& A,
                         const BodyState& B,
                         const EvaluatedBody& evalA,
                         const EvaluatedBody& evalB,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        const Eigen::Vector3d rA = evalA.q * anchorA;
        const Eigen::Vector3d rB = evalB.q * anchorB;

        if (A.global_idx >= 0) {
            triplets.emplace_back(rowOffset + 0, A.global_idx + 0, 1.0);
            triplets.emplace_back(rowOffset + 1, A.global_idx + 1, 1.0);
            triplets.emplace_back(rowOffset + 2, A.global_idx + 2, 1.0);

            const Eigen::Matrix3d drA = -crossMat(rA);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    triplets.emplace_back(rowOffset + i, A.global_idx + 3 + j, drA(i, j));
                }
            }
        }

        if (B.global_idx >= 0) {
            triplets.emplace_back(rowOffset + 0, B.global_idx + 0, -1.0);
            triplets.emplace_back(rowOffset + 1, B.global_idx + 1, -1.0);
            triplets.emplace_back(rowOffset + 2, B.global_idx + 2, -1.0);

            const Eigen::Matrix3d drB = crossMat(rB);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    triplets.emplace_back(rowOffset + i, B.global_idx + 3 + j, drB(i, j));
                }
            }
        }
    }
};

struct PointOnAxisPrimitive {
    Eigen::Vector3d anchorA = Eigen::Vector3d::Zero();
    Eigen::Vector3d anchorB = Eigen::Vector3d::Zero();
    Eigen::Vector3d uA = Eigen::Vector3d::UnitY();
    Eigen::Vector3d vA = Eigen::Vector3d::UnitZ();

    Eigen::Vector2d value(const EvaluatedBody& A, const EvaluatedBody& B) const {
        const Eigen::Vector3d rA = A.q * anchorA;
        const Eigen::Vector3d rB = B.q * anchorB;
        const Eigen::Vector3d d = (B.p + rB) - (A.p + rA);
        const Eigen::Vector3d uAw = A.q * uA;
        const Eigen::Vector3d vAw = A.q * vA;
        return Eigen::Vector2d(d.dot(uAw), d.dot(vAw));
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& A,
                         const BodyState& B,
                         const EvaluatedBody& evalA,
                         const EvaluatedBody& evalB,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        const Eigen::Vector3d rA = evalA.q * anchorA;
        const Eigen::Vector3d rB = evalB.q * anchorB;
        const Eigen::Vector3d d = (evalB.p + rB) - (evalA.p + rA);
        const Eigen::Vector3d uAw = evalA.q * uA;
        const Eigen::Vector3d vAw = evalA.q * vA;

        const auto addOne = [&](int row, const Eigen::Vector3d& dir) {
            if (A.global_idx >= 0) {
                triplets.emplace_back(row, A.global_idx + 0, -dir.x());
                triplets.emplace_back(row, A.global_idx + 1, -dir.y());
                triplets.emplace_back(row, A.global_idx + 2, -dir.z());

                const Eigen::Vector3d gA = dir.cross(rA) + dir.cross(d);
                for (int i = 0; i < 3; ++i) {
                    triplets.emplace_back(row, A.global_idx + 3 + i, gA(i));
                }
            }

            if (B.global_idx >= 0) {
                triplets.emplace_back(row, B.global_idx + 0, dir.x());
                triplets.emplace_back(row, B.global_idx + 1, dir.y());
                triplets.emplace_back(row, B.global_idx + 2, dir.z());

                const Eigen::Vector3d gB = rB.cross(dir);
                for (int i = 0; i < 3; ++i) {
                    triplets.emplace_back(row, B.global_idx + 3 + i, gB(i));
                }
            }
        };

        addOne(rowOffset + 0, uAw);
        addOne(rowOffset + 1, vAw);
    }
};

struct DirectionPrimitive {
    Eigen::Vector3d axisA = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d uB = Eigen::Vector3d::UnitX();
    Eigen::Vector3d vB = Eigen::Vector3d::UnitY();

    Eigen::Vector2d value(const EvaluatedBody& A, const EvaluatedBody& B) const {
        const Eigen::Vector3d nA = A.q * axisA;
        const Eigen::Vector3d uBw = B.q * uB;
        const Eigen::Vector3d vBw = B.q * vB;
        return Eigen::Vector2d(nA.dot(uBw), nA.dot(vBw));
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& A,
                         const BodyState& B,
                         const EvaluatedBody& evalA,
                         const EvaluatedBody& evalB,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        const Eigen::Vector3d nA = evalA.q * axisA;
        const Eigen::Vector3d uBw = evalB.q * uB;
        const Eigen::Vector3d vBw = evalB.q * vB;

        const auto addOne = [&](int row, const Eigen::Vector3d& bDir) {
            if (A.global_idx >= 0) {
                const Eigen::Vector3d gA = nA.cross(bDir);
                for (int i = 0; i < 3; ++i) {
                    triplets.emplace_back(row, A.global_idx + 3 + i, gA(i));
                }
            }
            if (B.global_idx >= 0) {
                const Eigen::Vector3d gB = bDir.cross(nA);
                for (int i = 0; i < 3; ++i) {
                    triplets.emplace_back(row, B.global_idx + 3 + i, gB(i));
                }
            }
        };

        addOne(rowOffset + 0, uBw);
        addOne(rowOffset + 1, vBw);
    }
};

struct AxisCrossPrimitive {
    Eigen::Vector3d axisA = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d axisB = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d value(const EvaluatedBody& A, const EvaluatedBody& B) const {
        const Eigen::Vector3d nA = A.q * axisA;
        const Eigen::Vector3d nB = B.q * axisB;
        return nA.cross(nB);
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& A,
                         const BodyState& B,
                         const EvaluatedBody& evalA,
                         const EvaluatedBody& evalB,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        const Eigen::Vector3d nA = evalA.q * axisA;
        const Eigen::Vector3d nB = evalB.q * axisB;

        if (A.global_idx >= 0) {
            const Eigen::Matrix3d dA = crossMat(nB) * crossMat(nA);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    triplets.emplace_back(rowOffset + i, A.global_idx + 3 + j, dA(i, j));
                }
            }
        }

        if (B.global_idx >= 0) {
            const Eigen::Matrix3d dB = -crossMat(nA) * crossMat(nB);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    triplets.emplace_back(rowOffset + i, B.global_idx + 3 + j, dB(i, j));
                }
            }
        }
    }
};

struct RollLockPrimitive {
    Eigen::Vector3d uA = Eigen::Vector3d::UnitX();
    Eigen::Vector3d vB = Eigen::Vector3d::UnitY();

    double value(const EvaluatedBody& A, const EvaluatedBody& B) const {
        const Eigen::Vector3d uAw = A.q * uA;
        const Eigen::Vector3d vBw = B.q * vB;
        return uAw.dot(vBw);
    }

    void addJacobianRow(int row,
                        const BodyState& A,
                        const BodyState& B,
                        const EvaluatedBody& evalA,
                        const EvaluatedBody& evalB,
                        std::vector<Eigen::Triplet<double>>& triplets) const {
        const Eigen::Vector3d uAw = evalA.q * uA;
        const Eigen::Vector3d vBw = evalB.q * vB;

        if (A.global_idx >= 0) {
            const Eigen::Vector3d gA = uAw.cross(vBw);
            for (int i = 0; i < 3; ++i) {
                triplets.emplace_back(row, A.global_idx + 3 + i, gA(i));
            }
        }
        if (B.global_idx >= 0) {
            const Eigen::Vector3d gB = vBw.cross(uAw);
            for (int i = 0; i < 3; ++i) {
                triplets.emplace_back(row, B.global_idx + 3 + i, gB(i));
            }
        }
    }
};

struct FixedPositionPrimitive {
    Eigen::Vector3d target = Eigen::Vector3d::Zero();

    Eigen::Vector3d value(const EvaluatedBody& body) const {
        return body.p - target;
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& body,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        if (body.global_idx < 0) {
            return;
        }
        triplets.emplace_back(rowOffset + 0, body.global_idx + 0, 1.0);
        triplets.emplace_back(rowOffset + 1, body.global_idx + 1, 1.0);
        triplets.emplace_back(rowOffset + 2, body.global_idx + 2, 1.0);
    }
};

struct FixedOrientationPrimitive {
    Eigen::Quaterniond target = Eigen::Quaterniond::Identity();

    Eigen::Vector3d value(const BodyState& body, const Eigen::VectorXd& x) const {
        if (body.global_idx < 0) {
            return Eigen::Vector3d::Zero();
        }
        const Eigen::Quaterniond dq_target = target * body.q_ref.conjugate();
        const Eigen::AngleAxisd aa(dq_target);
        const Eigen::Vector3d theta_target = aa.angle() * aa.axis();
        const Eigen::Vector3d theta = x.segment<3>(body.global_idx + 3);
        return theta - theta_target;
    }

    void addJacobianRows(int rowOffset,
                         const BodyState& body,
                         std::vector<Eigen::Triplet<double>>& triplets) const {
        if (body.global_idx < 0) {
            return;
        }
        triplets.emplace_back(rowOffset + 0, body.global_idx + 3, 1.0);
        triplets.emplace_back(rowOffset + 1, body.global_idx + 4, 1.0);
        triplets.emplace_back(rowOffset + 2, body.global_idx + 5, 1.0);
    }
};

} // namespace NexDynIPC::Dynamics::JointPrimitives
