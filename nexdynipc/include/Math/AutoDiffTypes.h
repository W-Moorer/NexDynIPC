/**
 * @file AutoDiffTypes.h
 * @brief Convenient type aliases and helper functions for automatic differentiation
 */

#ifndef NEXDYNIPC_MATH_AUTODIFF_TYPES_HPP
#define NEXDYNIPC_MATH_AUTODIFF_TYPES_HPP

#include <NexDynIPC/Math/AutoDiff.h>
#include <vector>
#include <cassert>

namespace NexDynIPC::Math {

/**
 * @brief Template class providing convenient type aliases for automatic differentiation
 * 
 * @tparam N Number of variables (use Eigen::Dynamic for runtime-sized)
 * @tparam maxN Maximum number of variables for fixed-size types
 */
template <int N, int maxN = N>
class AutodiffType {
public:
    typedef Eigen::Matrix<double, N, 1, Eigen::ColMajor, maxN, 1> VectorNd;
    typedef Eigen::Matrix<double, N, N, Eigen::ColMajor, maxN, maxN> MatrixNd;

    typedef DScalar1<double, VectorNd> DDouble1;
    typedef DScalar2<double, VectorNd, MatrixNd> DDouble2;

    typedef Eigen::Matrix<DDouble1, Eigen::Dynamic, Eigen::Dynamic> D1MatrixXd;
    typedef Eigen::Matrix<DDouble2, Eigen::Dynamic, Eigen::Dynamic> D2MatrixXd;

    typedef Eigen::Matrix<DDouble1, Eigen::Dynamic, 1> D1VectorXd;
    typedef Eigen::Matrix<DDouble2, Eigen::Dynamic, 1> D2VectorXd;

    typedef Eigen::Matrix<DDouble1, 3, 1> D1Vector3d;
    typedef Eigen::Matrix<DDouble2, 3, 1> D2Vector3d;
    typedef Eigen::Matrix<DDouble1, 2, 1> D1Vector2d;
    typedef Eigen::Matrix<DDouble2, 2, 1> D2Vector2d;

    inline static void activate() {
        static_assert(N >= 0, "N must be non-negative");
        DiffScalarBase::setVariableCount(N);
    }

    inline static void activate(size_t variableCount) {
        assert(N == Eigen::Dynamic && "Use activate() for compile-time sized types");
        DiffScalarBase::setVariableCount(variableCount);
    }

    template <typename T>
    inline static Eigen::Matrix<T, Eigen::Dynamic, 1> dTvars(const size_t i, const Eigen::VectorXd& v) {
        Eigen::Matrix<T, Eigen::Dynamic, 1> vec;
        vec.resize(v.rows());
        for (int r = 0; r < v.rows(); r++) {
            vec[r] = T(i + r, v[r]);
        }
        return vec;
    }

    inline static D1VectorXd d1vars(const size_t i, const Eigen::VectorXd& v) {
        return dTvars<DDouble1>(i, v);
    }

    inline static D2VectorXd d2vars(const size_t i, const Eigen::VectorXd& v) {
        return dTvars<DDouble2>(i, v);
    }

    template <typename DVectorXd>
    inline static Eigen::VectorXd get_value(const DVectorXd& x) {
        Eigen::VectorXd val(x.rows());
        for (int i = 0; i < x.rows(); ++i) {
            val(i) = x(i).getValue();
        }
        return val;
    }

    template <typename DVectorXd>
    inline static Eigen::MatrixXd get_gradient(const DVectorXd& x) {
        Eigen::MatrixXd grad(x.rows(), DiffScalarBase::getVariableCount());
        for (int i = 0; i < x.rows(); ++i) {
            grad.row(i) = x(i).getGradient();
        }
        return grad;
    }

    inline static std::vector<MatrixNd> get_hessian(const D2VectorXd& x) {
        std::vector<MatrixNd> hess;
        hess.reserve(x.rows());
        for (int i = 0; i < x.rows(); ++i) {
            hess.push_back(x(i).getHessian());
        }
        return hess;
    }
};

template <typename DScalar, typename Scalar = typename DScalar::Scalar>
const Scalar& get_value(const DScalar& dscalar) {
    return dscalar.getValue();
}

template <typename DScalar, typename Gradient = typename DScalar::Gradient>
const Gradient& get_gradient(const DScalar& dscalar) {
    return dscalar.getGradient();
}

template <typename Scalar, typename Gradient,
          typename Hessian = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>
const Hessian& get_hessian(const DScalar1<Scalar, Gradient>& dscalar1) {
    throw std::runtime_error("DScalar1 does not compute the hessian");
}

template <typename Scalar, typename Gradient, typename Hessian>
const Hessian& get_hessian(const DScalar2<Scalar, Gradient, Hessian>& dscalar2) {
    return dscalar2.getHessian();
}

using DynamicAutodiff = AutodiffType<Eigen::Dynamic>;
using Autodiff2D = AutodiffType<3>;
using Autodiff3D = AutodiffType<6>;
using AutodiffRigidBody = AutodiffType<6>;

} // namespace NexDynIPC::Math

#endif // NEXDYNIPC_MATH_AUTODIFF_TYPES_HPP
