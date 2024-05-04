#ifndef CUSTOM_SERIAL_DRIVER_H
#define CUSTOM_SERIAL_DRIVER_H

#include <custom_serial_interfaces/srv/detail/send_package__struct.hpp>
#include <memory>
#include <rclcpp/service.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <custom_serial_interfaces/msg/receive.hpp>
#include <custom_serial_interfaces/srv/send_package.hpp>

namespace custom_serial {
class SerialDriverNode : public rclcpp::Node {
  static constexpr int BUFF_LEN = 64;

public:
  SerialDriverNode(const rclcpp::NodeOptions &options);

private:
  // Serial port
  std::string m_device_name;
  std::unique_ptr<IoContext> m_io_ctx;
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_driver;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_port_cfg;
  std::thread m_receive_thread;
  // Publisher
  rclcpp::Publisher<custom_serial_interfaces::msg::Receive>::SharedPtr
      m_receive_pub;
  // Service
  rclcpp::Service<custom_serial_interfaces::srv::SendPackage>::SharedPtr
      m_send_service;

  void initParamenters();

  void reopen();

  void receiveData();

  void sendData(uint8_t func_code, uint16_t id, uint16_t len,
                const uint8_t *data);
};
} // namespace custom_serial

#endif
