#include <controller_io/controller_io_node.h>

#include <cmath>
#include <cstring>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

namespace armor_auto_aim {
using Receive = custom_serial_interfaces::msg::Receive;
using SendPackage = custom_serial_interfaces::srv::SendPackage;

ControllerIONode::ControllerIONode(const rclcpp::NodeOptions &options)
    : Node("controller_io", options) {
  // Subsciption
  m_serial_sub = this->create_subscription<Receive>(
      "/custom_serial/receive", rclcpp::SensorDataQoS(),
      std::bind(&ControllerIONode::serialHandle, this, std::placeholders::_1));
  // Client
  m_serial_cli = this->create_client<SendPackage>("/custom_serial/send");
  m_send_thread = std::thread([this]() -> void {
    using namespace std::chrono_literals;
    std::string msg = "Controller test";
    int len = msg.size() + 1;
    while (true) {
      std::this_thread::sleep_for(500ms);
      auto request = std::make_shared<SendPackage::Request>();
      request->id = mPCId;
      request->func_code = 1;
      request->len = len;
      request->data.resize(request->len); // NOTE: must to be resize
      std::memcpy(request->data.data(), msg.c_str(), len);

      while (!m_serial_cli->wait_for_service(500ms))
        RCLCPP_WARN(this->get_logger(), "wait service timeout!");
      if (rclcpp::ok()) {
        m_serial_cli->async_send_request(request);
      }
    }
  });
}

void ControllerIONode::serialHandle(const Receive::SharedPtr serial_msg) {
  RCLCPP_INFO(this->get_logger(), "fun: %d; id: %d; len: %d; data: %s;",
              serial_msg->func_code, serial_msg->id, serial_msg->len,
              serial_msg->data.data());
}
} // namespace armor_auto_aim

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ControllerIONode)
