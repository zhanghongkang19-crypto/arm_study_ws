#include "robot_motion/pseudo_inverse_ik.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_msgs/msg/arm_position_cmd.hpp"
#include <memory>
#include <vector>
#include <chrono>
#include <filesystem>
#include <iostream>


using namespace std::chrono_literals;
namespace fs = std::filesystem;

class RobotMonitor : public rclcpp::Node
{
public:
    RobotMonitor(): Node("robot_monitor")
    {
        for(int i=0;i<8;i++)
        {
            cmd[i]=0.0;
        }

        std::string package_path =
    ament_index_cpp::get_package_share_directory("robot_description");

// 拼接 URDF 路径
fs::path urdf_path =
    fs::path(package_path) / "urdf" / "your_robot.urdf";

        pseudoInverseIK_ = new PseudoInverseIK()
        // 发布机械臂控制指令
        cmd_pub_ = this->create_publisher<robot_msgs::msg::ArmPositionCmd>( "/rebot/joint_group_cmd", 10);
        // 订阅机械臂状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>( "/joint_states", 10,std::bind( &RobotMonitor::jointStateCallback, this,std::placeholders::_1));
        // 定时发送控制命令
        timer_ = this->create_wall_timer(500ms,std::bind(&RobotMonitor::timerCallback,this));
        RCLCPP_INFO(this->get_logger(),"Robot monitor node started.");
    }



private:
    // ============================
    // 周期发送控制命令
    // ============================
    void timerCallback()
    {

       

        cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),"Publish joint command");
    }




    // ============================
    // 接收关节状态
    // ============================
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO( this->get_logger(),"Received joint state");

        for(size_t i=0;i<msg->name.size();i++)
        {
            double pos=0.0;
            if(i < msg->position.size())
            {
                pos=msg->position[i];
            }
            RCLCPP_INFO( this->get_logger(),"%s : %.4f", msg->name[i].c_str(), pos);
        }
    }



private:
    double cmd[8];
    PseudoInverseIK pseudoInverseIK_;
    rclcpp::Publisher<robot_msgs::msg::ArmPositionCmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
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
