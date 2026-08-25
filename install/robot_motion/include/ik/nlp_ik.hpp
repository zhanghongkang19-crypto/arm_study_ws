#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <casadi/casadi.hpp>

using Vector7d = Eigen::Matrix<double,7,1>;

class NlpIK
{
public:
    using Scalar   = double;
    using ADScalar = casadi::SX;
    using Model   = pinocchio::ModelTpl<Scalar>;
    using Data    = pinocchio::DataTpl<Scalar>;
    using ADModel = pinocchio::ModelTpl<ADScalar>;
    using ADData  = pinocchio::DataTpl<ADScalar>;

public:
    NlpIK(
        const std::string& urdf_path,
        const std::string& base_link,
        const std::string& end_effector_link
    );

    // FK返回: x y z qx qy qz qw
    Eigen::Matrix<double,7,1>   get_fk(const Eigen::VectorXd& joint_angles, const std::string& link_name);

    // IK
    bool get_ipopt_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, const std::string& link_name, Eigen::VectorXd& result_joint_angles);

    bool get_sqp_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, const std::string& link_name, Eigen::VectorXd& result_joint_angles);

private:
    Eigen::VectorXd calc_delta_x(const Eigen::Matrix4d& cur_pose, const Eigen::Matrix4d& tar_pose);
    Eigen::VectorXd computePoseError(const pinocchio::SE3& current,const pinocchio::SE3& target);
    bool solveQP(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,const Eigen::VectorXd& dq_lb,const Eigen::VectorXd& dq_ub,Eigen::VectorXd& dq);

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;
    casadi::Opti opti_;
    casadi::MX translational_cost_, rotational_cost_, regularization_cost_, smooth_cost_;
    casadi::MX var_q_, var_q_last_, param_tf_ ;

    //sqp相关
    double position_weight_;
    double rotation_weight_;
    double regularization_;
    int max_iterations_;
    double tolerance_;
};