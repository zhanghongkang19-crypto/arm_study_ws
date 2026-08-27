#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "ik/nlp_ik.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "robot_msgs/msg/arm_position_cmd.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class MoveItTargetPlanner : public rclcpp::Node
{
public:

    MoveItTargetPlanner(): Node("moveit_target_planner"),nlpIK_("reBot_B601_DM_with_gripper.urdf", "base_link", "link6")
    {
        target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/target_pose",10,
                            std::bind(&MoveItTargetPlanner::targetPoseCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<robot_msgs::msg::ArmPositionCmd>( "/rebot/joint_group_cmd", 1);
         // 订阅机械臂状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>( "/joint_states", 10,std::bind( &MoveItTargetPlanner::jointStateCallback, this,std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "MoveIt Target Planner started.");
        RCLCPP_INFO(this->get_logger(),"Waiting for /target_pose ...");
    }

    void initializeMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        move_group_->setPlanningTime(5.0);  //规划器搜索时间
        move_group_->setNumPlanningAttempts(5); // 规划尝试次数
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);    //按照配置文件允许的最大加速度的 100% 来规划。
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setEndEffectorLink("link6");

        RCLCPP_INFO(this->get_logger(),"MoveGroup initialized.");
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s",move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO( this->get_logger(), "End effector: %s", move_group_->getEndEffectorLink().c_str());
    }


    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //RCLCPP_INFO( this->get_logger(),"Received joint state");
        std::lock_guard<std::mutex> lock_guard(mutex_);
        joint_positions_.clear();
        for(size_t i=0;i<msg->name.size();i++)
        {
          
            if(i < msg->position.size())
            {
                double pos = msg->position[i];
                joint_positions_.push_back(pos);
            }
           // RCLCPP_INFO( this->get_logger(),"%s : %.4f", msg->name[i].c_str(), joint_positions_[i]);
        }
    }

private:

    void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Received target pose: " "x=%.3f y=%.3f z=%.3f", msg->position.x, msg->position.y, msg->position.z);
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(),"MoveGroup is not initialized.");
            return;
        }
        
        robot_msgs::msg::ArmPositionCmd pub_msg;
        pub_msg.position.resize(8);
        std::unique_lock<std::mutex> lock(mutex_);
        if(joint_positions_.empty()){
            lock.unlock();
            return;
        }
        Eigen::VectorXd joint_positions = Eigen::Map<Eigen::VectorXd>(joint_positions_.data(), joint_positions_.size());
        lock.unlock();
        Eigen::VectorXd result_joint_angles;
        Eigen::VectorXd target_pose(7);

        target_pose[0] = msg->position.x;
        target_pose[1] = msg->position.y;
        target_pose[2] = msg->position.z;

        target_pose[3] = msg->orientation.x;
        target_pose[4] = msg->orientation.y;
        target_pose[5] = msg->orientation.z;
        target_pose[6] = msg->orientation.w;
        bool success = nlpIK_.get_ipopt_ik(joint_positions, target_pose, "link6", result_joint_angles);     
        if(success)
        {
            std:: cout << "求逆解成功， 目标位置为：" ;
            for(int i=0;i<result_joint_angles.size();i++)
            {
                std::cout << result_joint_angles[i] << "  ";
                pub_msg.position[i] = result_joint_angles[i];
                 
            }
            rclcpp::Rate rate(50.0);
            for(int i=0;i<500;i++){
                cmd_pub_->publish(pub_msg);
                rate.sleep();
            }
            
            std::cout << std::endl;
        }
        else
        {
            std::cout << "求逆解失败" << std::endl;
        }

        move_group_->setPoseTarget(*msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO( this->get_logger(),"Start planning...");

        auto result = move_group_->plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(),"Motion planning failed.");
            move_group_->clearPoseTargets();
            return;
        }

        rclcpp::Rate rate(50.0);
        const auto& trajectory = plan.trajectory.joint_trajectory;
        const auto& points = trajectory.points;
        for (size_t i = 0; i + 1 < points.size(); ++i)
        {
            const auto& q0 = points[i].positions;
            const auto& q1 = points[i + 1].positions;

            // // 100ms / 20ms = 5
            // for (int k = 0; k <= 5; ++k)
            // {
            //     double alpha = static_cast<double>(k) / 5.0;
            //     std::vector<double> q(q0.size());
            //     for (size_t j = 0; j < q0.size(); ++j)
            //     {
            //         pub_msg.position[j] = q0[j] + alpha * (q1[j] - q0[j]);
            //     }
            //     cmd_pub_->publish(pub_msg);
            //     rate.sleep();
                   
            // }

        }

        std::cout << "这是moveit规划出来的最后一个点位：" ;
        const auto& q = points[points.size()-1].positions;
        for (size_t j = 0; j < q.size(); ++j)
        {
            std::cout << q[j]  << " ";
         }

        
    }





private:
    rclcpp::Publisher<robot_msgs::msg::ArmPositionCmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    NlpIK   nlpIK_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::mutex mutex_;
    std::vector<double> joint_positions_;
};


int main(
    int argc,
    char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItTargetPlanner>();
    node->initializeMoveGroup();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}