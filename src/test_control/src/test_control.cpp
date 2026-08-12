#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_msgs/msg/arm_position_cmd.hpp"


using namespace std::chrono_literals;


class RobotMonitor : public rclcpp::Node
{
public:
    RobotMonitor(): Node("robot_monitor")
    {
        for(int i=0;i<8;i++)
        {
            cmd[i]=0.0;
        }


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

        // 测试控制joint3 joint4
        if(cmd[3] >-1.0)
        {
            cmd[3]-=0.1;
        }


        if(cmd[2] > -1.0)
        {
            cmd[2]-=0.1;
        }


        robot_msgs::msg::ArmPositionCmd msg;
        msg.position.resize(8);


        for(int i=0;i<8;i++)
        {
            msg.position[i]=cmd[i];
        }


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
