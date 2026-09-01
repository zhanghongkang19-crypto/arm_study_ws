#include "mpc_control/mpc_controller.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mpc_control::MpcController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mpc_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
