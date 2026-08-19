#include "ik/pseudo_inverse_ik.hpp" 
#include <ament_index_cpp/get_package_share_directory.hpp>
 
PseudoInverseIK::PseudoInverseIK(const std::string& urdf_path,const std::string& base_link,const std::string& end_effector_link)
{
    
    std::string package_path = ament_index_cpp::get_package_share_directory("rebot_description");
    std::string  configuration_path_ = package_path + "/urdf/";
    configuration_path_ = configuration_path_ + urdf_path;
	pinocchio::urdf::buildModel(configuration_path_, model_);
	std::cout << "Model loaded. nq = " << model_.nq << ", nv = " << model_.nv << std::endl;

    data_ = std::make_unique<pinocchio::Data>(model_);
    ee_frame_id_ = model_.getFrameId(end_effector_link);
    iteration_limit_ = 1000;
    tolerance_ = 1e-6;
}

Eigen::Matrix<double,7,1>  PseudoInverseIK::get_fk(const Eigen::VectorXd& joint_angles, const std::string& link_name)
{
    pinocchio::forwardKinematics(model_, *data_, joint_angles);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::FrameIndex frame_id = model_.getFrameId(link_name);
    const auto& frame_placement = data_->oMf[frame_id];
    Eigen::Matrix<double,7,1> pose;
    pose.head<3>() = frame_placement.translation();
    Eigen::Quaterniond q(frame_placement.rotation());
    pose.tail<4>() << q.x(), q.y(), q.z(), q.w();
    return pose;
}


bool PseudoInverseIK::get_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{
    for(int i=0; i<iteration_limit_; ++i)
    {
        Eigen::Matrix<double,7,1> current_pose = get_fk(initial_joint_angles, link_name);
        Eigen::Matrix4d cur_pose_matrix = Eigen::Matrix4d::Identity();
        cur_pose_matrix.block<3, 1>(0, 3) = current_pose.head<3>();
        Eigen::Quaterniond q(current_pose.tail<4>());
        cur_pose_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

        Eigen::Matrix4d tar_pose_matrix = Eigen::Matrix4d::Identity();
        tar_pose_matrix.block<3, 1>(0, 3) = target_pose.head<3>();
        Eigen::Quaterniond q_target(target_pose.tail<4>());
        tar_pose_matrix.block<3, 3>(0, 0) = q_target.toRotationMatrix();

        Eigen::VectorXd delta_x = calc_delta_x(cur_pose_matrix, tar_pose_matrix);

        if(delta_x.norm() < tolerance_){
            result_joint_angles = initial_joint_angles;
            return true;
        }
            
        pinocchio::computeJointJacobians(model_, *data_, initial_joint_angles);
        pinocchio::updateFramePlacements(model_, *data_);
        Eigen::MatrixXd J = pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
        
        //使用 Eigen 的完全正交分解计算 Jacobian 的 Moore-Penrose 伪逆。
        Eigen::MatrixXd J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        
        Eigen::VectorXd delta_theta = J_pseudo_inverse * delta_x;
        
        initial_joint_angles += delta_theta;
    }
    return false;
    
}


bool PseudoInverseIK::get_null_space_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{
    for(int i=0; i<iteration_limit_; ++i)
    {
        Eigen::Matrix<double,7,1> current_pose = get_fk(initial_joint_angles, link_name);
        Eigen::Matrix4d cur_pose_matrix = Eigen::Matrix4d::Identity();
        cur_pose_matrix.block<3, 1>(0, 3) = current_pose.head<3>();
        Eigen::Quaterniond q(current_pose.tail<4>());
        cur_pose_matrix.block<3, 3>(0, 0) = q.toRotationMatrix();

        Eigen::Matrix4d tar_pose_matrix = Eigen::Matrix4d::Identity();
        tar_pose_matrix.block<3, 1>(0, 3) = target_pose.head<3>();
        Eigen::Quaterniond q_target(target_pose.tail<4>());
        tar_pose_matrix.block<3, 3>(0, 0) = q_target.toRotationMatrix();

        
        Eigen::VectorXd delta_x = calc_delta_x(cur_pose_matrix, tar_pose_matrix);


        if(delta_x.norm() < tolerance_){
            result_joint_angles = initial_joint_angles;
            for (int i = 0; i < result_joint_angles.size(); ++i) {
                result_joint_angles(i) = std::clamp(
                    result_joint_angles(i),
                    model_.lowerPositionLimit(i),
                    model_.upperPositionLimit(i)
                );
            }

            return true;
        }
        
       

        pinocchio::computeJointJacobians(model_, *data_, initial_joint_angles);
        pinocchio::updateFramePlacements(model_, *data_);
        Eigen::MatrixXd J = pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);

        Eigen::MatrixXd J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        // double lambda = 0.01;
        // Eigen::MatrixXd I =Eigen::MatrixXd::Identity(J.rows(), J.rows());

        // Eigen::MatrixXd J_pseudo_inverse = J.transpose() *(J * J.transpose() + lambda * lambda * I).inverse();
        
        
        Eigen::VectorXd delta_theta = J_pseudo_inverse * delta_x;
     
        //零空间
        auto I = Eigen::MatrixXd::Identity(8, 8);
        auto N = I-J_pseudo_inverse*J;
        Eigen::Vector<double,8> q_t;
        q_t(0) =    0   -   initial_joint_angles(0);
        q_t(1) =    0   -   initial_joint_angles(1);
        q_t(2) =    0   -   initial_joint_angles(2);
        q_t(3) =    0   -   initial_joint_angles(3);
        q_t(4) =    0   -   initial_joint_angles(4);
        q_t(5) =    0   -   initial_joint_angles(5);
        q_t(6) =    0   -   initial_joint_angles(6);
        q_t(7) =    0   -   initial_joint_angles(7);

        initial_joint_angles  += delta_theta+ 0.1*N* q_t;
        for (int i = 0; i < initial_joint_angles.size(); ++i) {
                initial_joint_angles(i) = std::clamp(
                    initial_joint_angles(i),
                    model_.lowerPositionLimit(i),
                    model_.upperPositionLimit(i)
                );
        }
    }
     return false;
}


Eigen::VectorXd PseudoInverseIK::calc_delta_x(const Eigen::Matrix4d& cur_pose, const Eigen::Matrix4d& tar_pose)
{
    Eigen::VectorXd twist;
    Eigen::Vector3d position_error = tar_pose.block<3, 1>(0, 3) - cur_pose.block<3, 1>(0, 3);
    Eigen::Matrix3d R = tar_pose.topLeftCorner<3, 3>() * cur_pose.topLeftCorner<3, 3>().inverse();
    Eigen::AngleAxisd angle_axis(R);
    Eigen::Vector3d rotation_error = angle_axis.angle() * angle_axis.axis();
	twist.resize(6);
    twist << position_error,rotation_error;
	twist *= 1;
    return twist;
}


