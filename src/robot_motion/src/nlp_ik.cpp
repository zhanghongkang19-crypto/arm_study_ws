#include "ik/nlp_ik.hpp" 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <qpOASES.hpp>
 
NlpIK::NlpIK(const std::string& urdf_path,const std::string& base_link,const std::string& end_effector_link) : 
      position_weight_(1.0),
      rotation_weight_(0.5),
      regularization_(1e-4),
      max_iterations_(100),
      tolerance_(1e-5)
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
 
	pinocchio::FrameIndex ad_ee_frame_id = ad_model.getFrameId(end_effector_link);
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
	casadi::SX p = eig3_to_sx(ad_data.oMf[ad_ee_frame_id].translation());

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

	casadi::SX Rot_err = casadi::SX::vertcat({
		rot_err(0),
		rot_err(1),
		rot_err(2),
	});
	casadi::Function rotational_error(
		"rotational_error",
		{ q_sym, Tf_sym },   // 输入
		{ Rot_err }          // 输出
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

	//约束
	Eigen::VectorXd q_min = model_.lowerPositionLimit;
	Eigen::VectorXd q_max = model_.upperPositionLimit;
	casadi::DM q_min_dm(q_min.size(), 1);
	casadi::DM q_max_dm(q_max.size(), 1);
	for (int i = 0; i < q_min.size(); ++i) {
		q_min_dm(i) = q_min(i);
		q_max_dm(i) = q_max(i);
	}
	opti_.subject_to(	//向优化问题中加入约束。
		opti_.bounded(q_min_dm, var_q_, q_max_dm)
	);
    

	casadi::Dict opts;	//创建 IPOPT 参数字典
	opts["expand"] = true;	//让 CasADi 在求解之前尝试展开表达式。
	opts["detect_simple_bounds"] = true;	//并尽可能把它作为变量上下界处理，而不是普通非线性约束。
	opts["calc_lam_p"] = false;	//控制 CasADi 是否计算参数相关的拉格朗日乘子。
	opts["print_time"] = false;
	opts["ipopt.sb"] = "yes";	//不打印 IPOPT 启动时的 Banner。
	opts["ipopt.print_level"] = 0;	//设置 IPOPT 输出级别。常见：0   几乎不输出
	opts["ipopt.max_iter"] = 30; 
	opts["ipopt.tol"] = 1e-4;	//POPT 有两套“满意程度”：这是 IPOPT 的主要收敛容差，严格收敛
	opts["ipopt.acceptable_tol"] = 5e-4;	//可接受收敛
	opts["ipopt.acceptable_iter"] = 5;		//如果连续若干次迭代都处于 acceptable 范围，就可以提前认为已经足够好了。
	opts["ipopt.max_wall_time"] = 0.015; // 限时10ms完成求解
	opts["ipopt.warm_start_init_point"] = "yes";	//使用上一次优化得到的结果作为这一次优化的初始信息。
	opts["ipopt.derivative_test"] = "none";		//关闭 IPOPT 的导数检查。
	opts["ipopt.jacobian_approximation"] = "exact";	//exact就是尽量使用 CasADi 自动微分产生的 Jacobian。
	// opts["ipopt.hessian_approximation"] = "limited-memory"; // 若需要
	// 设置求解器
	opti_.solver("ipopt", opts);

	std::vector<double> ref_j = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	casadi::DM ref_data = casadi::DM::zeros(model_.nq, 1);
	ref_data = ref_j;
	opti_.set_value(q_ref, ref_data);

	std::vector<double> w_j = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
	casadi::DM w_data = casadi::DM::zeros(model_.nq, 1);
	w_data = w_j;
	opti_.set_value(joint_w, w_data);

	opti_.minimize(50.0 * translational_cost_ + 0.5 * rotational_cost_
					+ 0.1 * smooth_cost_ + 0.06 * regularization_cost_
				);

}

Eigen::Matrix<double,7,1>  NlpIK::get_fk(const Eigen::VectorXd& joint_angles, const std::string& link_name)
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


bool NlpIK::get_ipopt_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{	
	Eigen::Matrix4d target_tf = Eigen::Matrix4d::Identity();

	// 位置
	target_tf.block<3, 1>(0, 3) = target_pose.head<3>();

	// 四元数
	Eigen::Quaterniond q(
		target_pose(6),  // qw
		target_pose(3),  // qx
		target_pose(4),  // qy
		target_pose(5)   // qz
	);

	q.normalize();

	// 旋转矩阵
	target_tf.block<3, 3>(0, 0) = q.toRotationMatrix();

	casadi::DM Tf_dm = casadi::DM::eye(4);
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			Tf_dm(i,j) = target_tf(i,j);
		}
	}

	//把 Eigen 的 initial_joint_angles 转成 std::vector<double>，再转成 CasADi 的 DM，作为优化变量的初始值。
	std::vector<double> start_q(8, 0);
	Eigen::VectorXd::Map(&start_q[0], 8) = initial_joint_angles;	//用 Eigen::Map 把 start_q 的内存映射成 Eigen 向量
	casadi::DM init_data = casadi::DM::zeros(model_.nq, 1);
	init_data = start_q;

	opti_.set_initial(var_q_, init_data);
	opti_.set_value(param_tf_, Tf_dm);
	opti_.set_value(var_q_last_, init_data);


	try{
        casadi::OptiSol sol = opti_.solve();
		casadi::DM q_sol = sol.value(var_q_);
		result_joint_angles.resize(8);
		for (int i = 0; i < 8; ++i)
			result_joint_angles(i) = static_cast<double>(q_sol(i));

		// std::cout << "\nq_sol = " << ik_jp << std::endl;

    }catch(const std::exception& e){
		std::cerr << "IPOPT IK failed: "  << e.what() << std::endl;
        return false;
    }

	return true;

}


bool NlpIK::get_sqp_ik(Eigen::VectorXd& initial_joint_angles, const Eigen::VectorXd& target_pose, 
            const std::string& link_name,Eigen::VectorXd& result_joint_angles)
{
	Eigen::VectorXd q = initial_joint_angles;

	Eigen::VectorXd q_min = model_.lowerPositionLimit;
	Eigen::VectorXd q_max = model_.upperPositionLimit;



    Eigen::Vector3d target_position =  target_pose.head<3>();
    Eigen::Quaterniond target_quat;
    target_quat.w() = target_pose[6];
    target_quat.x() = target_pose[3];
    target_quat.y() = target_pose[4];
    target_quat.z() = target_pose[5];
    target_quat.normalize();

    pinocchio::SE3 target(target_quat.toRotationMatrix(),target_position);

    for (int iter = 0;iter < max_iterations_;++iter)
    {
        pinocchio::forwardKinematics(model_,*data_,q);
        pinocchio::updateFramePlacements(model_,*data_);
        pinocchio::SE3 current = data_->oMf[ee_frame_id_];

        Eigen::VectorXd e = computePoseError(current,target);
        double error_norm =e.norm();	//L2 范数

        std::cout << "iter = "<< iter << " error = " << error_norm << std::endl;

        if (error_norm < tolerance_)
        {
            result_joint_angles = q;
            std::cout << "SQP IK converged!" << std::endl;
            return true;
        }

        Eigen::MatrixXd J(6, model_.nv);	//因为三维空间中的三维刚体共有 6 个自由度，所以为6
        pinocchio::computeFrameJacobian( model_,*data_,q,ee_frame_id_,pinocchio::LOCAL,J);

        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6, 6);	// 1. 创建一个 6x6 的双精度浮点全零矩阵 W
        W.block<3,3>(0,0).setIdentity();	// 2. 将左上角 3x3 的子块设置为单位矩阵 (Identity)
        W.block<3,3>(3,3).setIdentity();
        W.block<3,3>(0,0) *= position_weight_;
        W.block<3,3>(3,3) *= rotation_weight_;

        Eigen::MatrixXd H = J.transpose()* W* J;
        H += regularization_ * Eigen::MatrixXd::Identity(model_.nv,model_.nv);
        Eigen::VectorXd g =-J.transpose() * W * e;
        Eigen::VectorXd dq_lb = q_min - q;
        Eigen::VectorXd dq_ub = q_max - q;

        // 防止一次更新太大
        const double max_step = 0.1;
        dq_lb = dq_lb.cwiseMax( Eigen::VectorXd::Constant( model_.nv,-max_step));
        dq_ub = dq_ub.cwiseMin(Eigen::VectorXd::Constant(model_.nv,max_step));

  

        Eigen::VectorXd dq;
        bool success = solveQP(H, g, dq_lb, dq_ub,dq);
        if (!success)
        {
            std::cerr << "QP failed!" << std::endl;
            return false;
        }

        double alpha = 1.0;

        q += alpha * dq;
        q = q.cwiseMax(q_min).cwiseMin(q_max);
    }

    result_joint_angles = q;

    return false;
}

bool NlpIK::solveQP(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,const Eigen::VectorXd& dq_lb,
    				const Eigen::VectorXd& dq_ub,Eigen::VectorXd& dq)
{
    const int n = H.rows();

    std::vector<qpOASES::real_t> H_qp(n * n);
    std::vector<qpOASES::real_t> g_qp(n);
    std::vector<qpOASES::real_t> lb(n);
    std::vector<qpOASES::real_t> ub(n);

    for (int i = 0; i < n; ++i)
    {
        g_qp[i] = g[i];

        lb[i] = dq_lb[i];
        ub[i] = dq_ub[i];

        for (int j = 0; j < n; ++j)
        {
            H_qp[i * n + j] = H(i, j);
        }
    }

    qpOASES::QProblem qp(n,0);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;	//把优化器的输出日志级别设置为“不打印”。
    qp.setOptions(options);

    int nWSR = 50;	//你最多迭代/更新工作集 50 次，尽量在这个次数内找到 QP 最优解
    qpOASES::returnValue ret =
        qp.init(
            H_qp.data(),
            g_qp.data(),
            nullptr,
            lb.data(),
            ub.data(),
            nullptr,
            nullptr,
            nWSR
        );

    if (ret != qpOASES::SUCCESSFUL_RETURN)
    {
        return false;
    }

    std::vector<qpOASES::real_t> x_opt(n);

    qp.getPrimalSolution(x_opt.data());
    dq.resize(n);
    for (int i = 0; i < n; ++i)
    {
        dq[i] = x_opt[i];
    }

    return true;
}


Eigen::VectorXd NlpIK::computePoseError(
    const pinocchio::SE3& current,
    const pinocchio::SE3& target)
{
    pinocchio::SE3 error_pose =current.actInv(target); //计算误差v,w
	pinocchio::Motion error = pinocchio::log6(error_pose);
    Eigen::VectorXd e(6);
    e.head<3>() = error.linear();
    e.tail<3>() = error.angular();

    return e;
}



Eigen::VectorXd NlpIK::calc_delta_x(const Eigen::Matrix4d& cur_pose, const Eigen::Matrix4d& tar_pose)
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
