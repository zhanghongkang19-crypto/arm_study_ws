#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "robot_msgs/msg/arm_position_cmd.hpp"


using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;


class MujocoController : public rclcpp::Node
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
    MujocoController():Node("mujoco_controller")
    {
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>( this, "/arm_controller/follow_joint_trajectory",
                            std::bind(&MujocoController::goal_callback,this,std::placeholders::_1,std::placeholders::_2),
                            std::bind(&MujocoController::cancel_callback,this,std::placeholders::_1),
                            std::bind(&MujocoController::accepted_callback,this,std::placeholders::_1));
        cmd_pub_ = this->create_publisher<robot_msgs::msg::ArmPositionCmd>( "/rebot/joint_group_cmd", 10);
        RCLCPP_INFO(get_logger(),"MoveIt arm_controller action ready");
    }

private:
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &,std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        auto traj = goal->trajectory;
        RCLCPP_INFO(get_logger(),"Receive trajectory");
        RCLCPP_INFO(get_logger(),"joint number=%ld",traj.joint_names.size());
        RCLCPP_INFO(get_logger(),"point number=%ld",traj.points.size());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(get_logger(),"Trajectory cancel");

        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void accepted_callback(std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(&MujocoController::execute,this,goal_handle).detach();
    }


    void execute(std::shared_ptr<GoalHandle> goal_handle)
    {
        std::cout << "1111"<< std::endl;
        auto goal = goal_handle->get_goal();
        auto trajectory = goal->trajectory;

        RCLCPP_INFO(get_logger(),"Start execute");

        for(size_t i=0; i<trajectory.points.size(); i++)
        {
            auto point = trajectory.points[i];
            std::vector<double> q = point.positions;

            robot_msgs::msg::ArmPositionCmd msg;
            msg.position.resize(8);
            for(int i=0; i<q.size(); i++){
                msg.position[i] = q[i];
            }
            cmd_pub_->publish(msg);


            RCLCPP_INFO(get_logger(),"point %ld  q=[%.3f %.3f %.3f %.3f %.3f %.3f]",i,q[0],q[1],q[2], q[3],q[4],q[5]); 
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);

        RCLCPP_INFO(get_logger(),"Trajectory finished");
    }

    rclcpp::Publisher<robot_msgs::msg::ArmPositionCmd>::SharedPtr cmd_pub_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};


int main(int argc,char ** argv)
{
    rclcpp::init( argc,argv);
    auto node =std::make_shared<MujocoController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}