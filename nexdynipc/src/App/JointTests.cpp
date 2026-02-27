#include "NexDynIPC/App/JointTests.h"
#include "NexDynIPC/Dynamics/Joints/SphericalJoint.h"
#include "NexDynIPC/Dynamics/Joints/PrismaticJoint.h"
#include "NexDynIPC/Dynamics/Joints/CylindricalJoint.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace NexDynIPC::Dynamics;

namespace NexDynIPC::App {

static void checkJacobian(const std::shared_ptr<Joint>& joint, const Eigen::VectorXd& x, const std::string& name) {
    Eigen::VectorXd C;
    joint->computeC(x, C);
    
    Eigen::SparseMatrix<double> J_sparse(joint->dim(), x.size());
    joint->computeJ(x, J_sparse);
    Eigen::MatrixXd J = J_sparse.toDense();

    Eigen::MatrixXd J_num(joint->dim(), x.size());
    double eps = 1e-7;
    
    for (int i = 0; i < x.size(); ++i) {
        Eigen::VectorXd x_p = x;
        x_p(i) += eps;
        Eigen::VectorXd C_p;
        joint->computeC(x_p, C_p);
        
        Eigen::VectorXd x_m = x;
        x_m(i) -= eps;
        Eigen::VectorXd C_m;
        joint->computeC(x_m, C_m);
        
        J_num.col(i) = (C_p - C_m) / (2 * eps);
    }
    
    double error = (J - J_num).norm();
    if (error < 1e-5) {
        std::cout << "[PASS] " << name << " Jacobian Check (Error: " << error << ")" << std::endl;
    } else {
        std::cout << "[FAIL] " << name << " Jacobian Check (Error: " << error << ")" << std::endl;
        std::cout << "Analytical:\n" << J << "\nNumerical:\n" << J_num << "\nDiff:\n" << (J - J_num) << std::endl;
    }
}

void JointTests::run() {
    std::cout << "Running Joint Tests..." << std::endl;
    
    // Setup Dummy IDs and offsets
    int idA = 0;
    int idB = 1;
    int idxA = 0; // x[0..5]
    int idxB = 6; // x[6..11]
    int n_dof = 12; // 2 bodies * 6 dof
    
    Eigen::Vector3d pA0 = Eigen::Vector3d(0,0,0);
    Eigen::Quaterniond qA0 = Eigen::Quaterniond::Identity();
    Eigen::Vector3d pB0 = Eigen::Vector3d(1,0,0);
    Eigen::Quaterniond qB0 = Eigen::Quaterniond::Identity();
    
    // --- Spherical Joint Test ---
    {
        // Anchor at (0.5, 0, 0)
        Eigen::Vector3d anchorA(0.5, 0, 0); 
        Eigen::Vector3d anchorB(-0.5, 0, 0);
        
        auto joint = std::make_shared<SphericalJoint>(idA, idB, anchorA, anchorB);
        
        // Randomize Reference State
        pA0 = Eigen::Vector3d::Random();
        pB0 = Eigen::Vector3d::Random();
        qA0 = Eigen::Quaterniond::UnitRandom();
        qB0 = Eigen::Quaterniond::UnitRandom();
        
        joint->updateState(idxA, idxB, pA0, qA0, pB0, qB0);
        
        // Check at x where theta=0 (but pos matches ref)
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n_dof); 
        x.segment<3>(idxA) = pA0;
        x.segment<3>(idxB) = pB0;
        
        checkJacobian(joint, x, "SphericalJoint");
    }

    // --- Prismatic Joint Test ---
    {
        Eigen::Vector3d anchorA(0,0,0);
        Eigen::Vector3d anchorB(0,0,0); 
        Eigen::Vector3d axisA(1,0,0);
        Eigen::Vector3d axisB(1,0,0);
        
        auto joint = std::make_shared<PrismaticJoint>(idA, idB, anchorA, anchorB, axisA, axisB);
        
        pA0 = Eigen::Vector3d::Random();
        pB0 = Eigen::Vector3d::Random();
        qA0 = Eigen::Quaterniond::UnitRandom();
        qB0 = Eigen::Quaterniond::UnitRandom();
        
        joint->updateState(idxA, idxB, pA0, qA0, pB0, qB0);
        
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n_dof);
        x.segment<3>(idxA) = pA0;
        x.segment<3>(idxB) = pB0;
        
        checkJacobian(joint, x, "PrismaticJoint");
    }

    // --- Cylindrical Joint Test ---
    {
        Eigen::Vector3d anchorA(0,0,0);
        Eigen::Vector3d anchorB(0,0,0); 
        Eigen::Vector3d axisA(1,0,0);
        Eigen::Vector3d axisB(1,0,0);
        
        auto joint = std::make_shared<CylindricalJoint>(idA, idB, anchorA, anchorB, axisA, axisB);
        
        pA0 = Eigen::Vector3d::Random();
        pB0 = Eigen::Vector3d::Random();
        qA0 = Eigen::Quaterniond::UnitRandom();
        qB0 = Eigen::Quaterniond::UnitRandom();
        
        joint->updateState(idxA, idxB, pA0, qA0, pB0, qB0);
        
        Eigen::VectorXd x = Eigen::VectorXd::Zero(n_dof);
        x.segment<3>(idxA) = pA0;
        x.segment<3>(idxB) = pB0;

        checkJacobian(joint, x, "CylindricalJoint");
    }
}

} // namespace NexDynIPC::App
