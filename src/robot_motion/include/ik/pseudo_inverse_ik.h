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

using Vector7d = Eigen::Matrix<double,7,1>;

class PseudoInverseIK
{
public:
    PseudoInverseIK(
        const std::string& urdf_path,
        const std::string& base_link,
        const std::string& end_effector_link
    );

    // FK返回: x y z qx qy qz qw
    Eigen::Matrix<double,7,1>   get_fk(const Eigen::VectorXd& joint_angles, const std::string& link_name);

    // IK
    bool get_ik(const Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, const std::string& link_name, Eigen::VectorXd& result_joint_angles);



private:
    Eigen::Matrix<double,7,1>   get_fk(const Eigen::VectorXd& joint_angles, const std::string& link_name);

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;
    int iteration_limit_;
    double tolerance_;
};