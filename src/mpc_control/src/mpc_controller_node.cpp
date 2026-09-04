#include "mpc_control/mpc_controller.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  // 初始化 ROS 2，并让 MpcController 一直处理订阅和定时器回调。
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mpc_control::MpcController>());
  } catch (const std::exception & error) {
    // 参数、URDF 或 OCS2 初始化失败时，打印原因并安全退出。
    RCLCPP_FATAL(rclcpp::get_logger("mpc_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  // 正常停止节点时释放 ROS 2 资源。
  rclcpp::shutdown();
  return 0;
}
