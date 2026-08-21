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


    //ipopt-----------------------------------------------------------------------
    ADModel ad_model = model_.cast<ADScalar>();//将model转为casadi符号模型
	ADData  ad_data(ad_model);

	casadi::SX q_sym = casadi::SX::sym("q", model_.nq);
	casadi::SX Tf_sym = casadi::SX::sym("tf",4,4);
	ADModel::ConfigVectorType q_ad(model_.nq);
	pinocchio::casadi::copy(q_sym, q_ad);

	pinocchio::forwardKinematics(ad_model, ad_data, q_ad);
	pinocchio::updateFramePlacements(ad_model, ad_data);

	pinocchio::FrameIndex ad_ee_frame_id = ad_model.getFrameId("end_effector_link");
    auto tcol = [](const casadi::SX& T4x4){
			return T4x4(casadi::Slice(0,3), casadi::Slice(3,4));  // 3x1
	};

	// 把 Eigen::Matrix<casadi::SX,3,1> 转成 casadi::SX(3x1)
	auto eig3_to_sx = [](const Eigen::Matrix<casadi::SX,3,1>& v){
			casadi::SX out(3,1);
			for(int i=0;i<3;++i) out(i) = v(i);
				return out;
	};

	// --- 构造平移误差 ---
	casadi::SX P = eig3_to_sx(ad_data.oMf[ad_ee_frame_id].translation());

	casadi::SX p_err = p - tcol(Tf_sym);
	//平移误差函数
	casadi::Function translational_error(
			"translational_error",
			{q_sym, Tf_sym,},
			{p_err}
	);

    const Eigen::Matrix<casadi::SX,3,3>& R = ad_data.oMf[ad_ee_frame_id].rotation();
	// 期望旋转矩阵（从 CasADi SX 转为 Eigen<casadi::SX>）
	Eigen::Matrix<casadi::SX,3,3> R_des;
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			R_des(i,j) = Tf_sym(i,j);
		}
	}

	Eigen::Matrix<casadi::SX,3,3> Rerr = R_des.transpose() * R;
	Eigen::Matrix<casadi::SX,3,3> skew = 0.5 * (Rerr - Rerr.transpose());
	Eigen::Matrix<casadi::SX,3,1> rot_err;
	rot_err << skew(2,1),skew(0,2),skew(1,0);

	casadi::SX rot_err = casadi::SX::vertcat({
		rot_err(0),
		rot_err(1),
		rot_err(2),
	});
	casadi::Function rotational_error(
		"rotational_error",
		{ q_sym, Tf_sym },   // 输入
		{ rot_err }          // 输出
	);

	var_q_ = opti_.variable(model_.nq);		//定义一个优化变量
	var_q_last_ = opti_.parameter(model_.nq);
	param_tf_ = opti_.parameter(4, 4);
	casadi::MX q_ref = opti_.parameter(model_.nq);
	casadi::MX joint_w = opti_.parameter(model_.nq);	//关节权重

	// 平移误差 cost
	casadi::MXVector out1 = translational_error(casadi::MXVector{var_q_, param_tf_});
	translational_cost_ = casadi::MX::sumsqr(out1.at(0));	//计算平方和。
	// 旋转误差 cost
	casadi::MXVector out2 = rotational_error(casadi::MXVector{var_q_, param_tf_});
	rotational_cost_ = casadi::MX::sumsqr(out2.at(0));
	// 正则化 cost（// 特定关节目标 / 加权跟踪）
	regularization_cost_ = casadi::MX::sumsqr(casadi::MX::diag(joint_w) * (var_q_ - q_ref));
	// 平滑 cost（与上一时刻解接近）
	smooth_cost_ = casadi::MX::sumsqr(var_q_ - var_q_last_);

    

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


bool PseudoInverseIK::get_ipopt_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{

}


bool PseudoInverseIK::get_sqp_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{
    
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
