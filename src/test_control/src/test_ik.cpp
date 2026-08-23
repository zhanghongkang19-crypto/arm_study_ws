#include "ik/pseudo_inverse_ik.hpp"
#include "ik/nlp_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_msgs/msg/arm_position_cmd.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <mutex>


using namespace std::chrono_literals;
namespace fs = std::filesystem;

class RobotMonitor : public rclcpp::Node
{
public:
    RobotMonitor(): Node("robot_monitor"), pseudoInverseIK_("reBot_B601_DM_with_gripper.urdf", "base_link", "link6"),   
            nlpIK_("reBot_B601_DM_with_gripper.urdf", "base_link", "link6")
    {
        for(int i=0;i<8;i++)
        {
            cmd[i]=0.0;
        }

        target_pose_ << 0.1, 0.0, 0.1908, 0.7173, 0.0001, 0.6967, -0.0001;

        // 拼接 URDF 路径
        fs::path urdf_path ="reBot_B601_DM_with_gripper.urdf";

        // 发布机械臂控制指令
        cmd_pub_ = this->create_publisher<robot_msgs::msg::ArmPositionCmd>( "/rebot/joint_group_cmd", 10);
        // 订阅机械臂状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>( "/joint_states", 10,std::bind( &RobotMonitor::jointStateCallback, this,std::placeholders::_1));
        // 定时发送控制命令
        //timer_ = this->create_wall_timer(20ms,std::bind(&RobotMonitor::timerCallback,this));    //伪逆+零空间
        timer_ = this->create_wall_timer(20ms,std::bind(&RobotMonitor::nlp_timerCallback,this));  //ipopt
        RCLCPP_INFO(this->get_logger(),"Robot monitor node started.");
    }



private:
    // ============================
    // 周期发送控制命令
    // ============================
    void timerCallback()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if(joint_positions_.empty()){
            lock.unlock();
            return;
        }
        Eigen::VectorXd joint_positions = Eigen::Map<Eigen::VectorXd>(joint_positions_.data(), joint_positions_.size());
        lock.unlock();
        Eigen::Matrix<double,7,1> pose = pseudoInverseIK_.get_fk(joint_positions, "link6");

        //RCLCPP_INFO(this->get_logger(),"Current pose: x: %.4f, y: %.4f, z: %.4f, qx: %.4f, qy: %.4f, qz: %.4f, qw: %.4f", pose(0), pose(1), pose(2), pose(3), pose(4), pose(5), pose(6));
        
        
        
        target_pose_(0) += 0.002; // Move 1 cm in x direction
        if(target_pose_(0) > 1){
            target_pose_(0) -=0.002;
        }
        
        Eigen::VectorXd result_joint_angles;
        robot_msgs::msg::ArmPositionCmd msg;
        msg.position.resize(8);
        bool success = pseudoInverseIK_.get_null_space_ik(joint_positions, target_pose_, "link6", result_joint_angles);
         
        if(success)
        {
            //RCLCPP_INFO(this->get_logger(),"IK solution found.");
            for(int i=0;i<result_joint_angles.size();i++)
            {
                msg.position[i] = result_joint_angles[i];
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),"IK solution not found.");       
            return;
        }



        cmd_pub_->publish(msg);
        //RCLCPP_INFO(this->get_logger(),"Publish joint command");
    }

    void nlp_timerCallback()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if(joint_positions_.empty()){
            lock.unlock();
            return;
        }
        Eigen::VectorXd joint_positions = Eigen::Map<Eigen::VectorXd>(joint_positions_.data(), joint_positions_.size());
        lock.unlock();
        Eigen::Matrix<double,7,1> pose = pseudoInverseIK_.get_fk(joint_positions, "link6");

        //RCLCPP_INFO(this->get_logger(),"Current pose: x: %.4f, y: %.4f, z: %.4f, qx: %.4f, qy: %.4f, qz: %.4f, qw: %.4f", pose(0), pose(1), pose(2), pose(3), pose(4), pose(5), pose(6));
        
        
        
        target_pose_(0) += 0.002; // Move 1 cm in x direction
        if(target_pose_(0) > 1){
            target_pose_(0) -=0.002;
        }
        
        Eigen::VectorXd result_joint_angles;
        robot_msgs::msg::ArmPositionCmd msg;
        msg.position.resize(8);
        //bool success = nlpIK_.get_ipopt_ik(joint_positions, target_pose_, "link6", result_joint_angles);
        bool success = nlpIK_.get_sqp_ik(joint_positions, target_pose_, "link6", result_joint_angles);     
    
        if(success)
        {
            //RCLCPP_INFO(this->get_logger(),"IK solution found.");
            for(int i=0;i<result_joint_angles.size();i++)
            {
                msg.position[i] = result_joint_angles[i];
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),"IK solution not found.");       
            return;
        }



        cmd_pub_->publish(msg);
        //RCLCPP_INFO(this->get_logger(),"Publish joint command");
    }


    // ============================
    // 接收关节状态
    // ============================
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
    Eigen::Matrix<double,7,1> target_pose_;
    double cmd[8];
    std::vector<double> joint_positions_;
    PseudoInverseIK pseudoInverseIK_;
    NlpIK   nlpIK_;
    rclcpp::Publisher<robot_msgs::msg::ArmPositionCmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mutex_;
};



int main(
    int argc,
    char ** argv)
{

    rclcpp::init(argc,argv);
    auto node =std::make_shared<RobotMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
