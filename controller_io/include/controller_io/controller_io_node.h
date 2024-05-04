#ifndef ARMOR_AUTO_AIM_CONTROLLER_IO_H
#define ARMOR_AUTO_AIM_CONTROLLER_IO_H

#include <rclcpp/rclcpp.hpp>

#include <custom_serial_interfaces/msg/receive.hpp>
#include <custom_serial_interfaces/srv/send_package.hpp>

namespace armor_auto_aim {
class ControllerIONode : public rclcpp::Node {
  // IPC
  static constexpr int mPCId = 0;
  // Controller
  static constexpr int mControllerId = 1;

public:
  ControllerIONode(const rclcpp::NodeOptions &options);

private:
  std::thread m_send_thread;
  // Subscription
  rclcpp::Subscription<custom_serial_interfaces::msg::Receive>::SharedPtr
      m_serial_sub;
  // Client
  rclcpp::Client<custom_serial_interfaces::srv::SendPackage>::SharedPtr
      m_serial_cli;

  // Callback
  void serialHandle(
      const custom_serial_interfaces::msg::Receive::SharedPtr serial_msg);
};
} // namespace armor_auto_aim

#endif
