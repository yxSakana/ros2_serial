#include <custom_serial_driver/serial_driver.h>

#include <cstdint>
#include <cstring>

#include <chrono>
#include <exception>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <fmt/format.h>

#include <custom_serial_driver/crc.h>

namespace custom_serial {
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
    : Node("serial_port", options), m_io_ctx(new IoContext(3)),
      m_driver(new drivers::serial_driver::SerialDriver(*m_io_ctx)) {
  initParamenters();
  RCLCPP_INFO(this->get_logger(), "%s", m_device_name.c_str());
  // Publisher
  m_receive_pub =
      this->create_publisher<custom_serial_interfaces::msg::Receive>(
          "/custom_serial/receive", rclcpp::SensorDataQoS());
  // Serivce
  using SendPackage = custom_serial_interfaces::srv::SendPackage;
  m_send_service = this->create_service<SendPackage>(
      "custom_serial/send",
      [this](const std::shared_ptr<SendPackage::Request> req,
             std::shared_ptr<SendPackage::Response> response) -> void {
        this->sendData(req->func_code, req->id, req->len, req->data.data());
        response->status = true;
      });
  // Open port && receive thread
  try {
    m_driver->init_port(m_device_name, *m_port_cfg);
    if (!m_driver->port()->is_open())
      m_driver->port()->open();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable open serial port: %s; %s",
                 m_device_name.c_str(), e.what());
    reopen();
  }
  m_receive_thread = std::thread(&SerialDriverNode::receiveData, this);
  m_receive_thread.detach();
}

void SerialDriverNode::initParamenters() {
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate;
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  m_device_name = this->declare_parameter("device_name", "/dev/ttyACM0");
  baud_rate = this->declare_parameter("baud_rate", 115200);
  const auto fc_str = this->declare_parameter("flow_control", "none");
  const auto pt_str = this->declare_parameter("parity", "none");
  const auto sb_str = this->declare_parameter("stop_bits", "1");

  if (fc_str == "none") {
    fc = FlowControl::NONE;
  } else if (fc_str == "hardware") {
    fc = FlowControl::HARDWARE;
  } else if (fc_str == "software") {
    fc = FlowControl::SOFTWARE;
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s ;<Usage> none OR hardware OR sofware",
                 fc_str.c_str());
  }

  if (pt_str == "none") {
    pt = Parity::NONE;
  } else if (pt_str == "odd") {
    pt = Parity::ODD;
  } else if (pt_str == "even") {
    pt = Parity::EVEN;
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s ;<Usage> none OR odd OR even",
                 pt_str.c_str());
  }

  if (sb_str == "1") {
    sb = StopBits::ONE;
  } else if (sb_str == "1.5") {
    sb = StopBits::ONE_POINT_FIVE;
  } else if (sb_str == "2") {
    sb = StopBits::TWO;
  } else {
    RCLCPP_ERROR(this->get_logger(), "%s; <Usage> 1 OR 1.5 OR 2",
                 sb_str.c_str());
  }

  m_port_cfg = std::make_unique<drivers::serial_driver::SerialPortConfig>(
      baud_rate, fc, pt, sb);
}

void SerialDriverNode::reopen() {
  RCLCPP_WARN(this->get_logger(), "try reopen serial port: %s",
              m_device_name.c_str());
  try {
    if (m_driver->port()->is_open())
      m_driver->port()->close();
    m_driver->port()->open();
    RCLCPP_INFO(this->get_logger(), "Success open serial port: %s",
                m_device_name.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable open serial port: %s; %s",
                 m_device_name.c_str(), e.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopen();
    }
  }
}

void SerialDriverNode::receiveData() {
  std::vector<uint8_t> buff(BUFF_LEN);
  uint8_t *pbuff;
  uint8_t *data;

  RCLCPP_INFO(this->get_logger(), "serial receive thread is running...");
  while (rclcpp::ok()) {
    try {
      m_driver->port()->receive(buff);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                            "receive error! %s", e.what());
    }
    if (buff[0] == 0x5A) {
      pbuff = buff.data();
      uint8_t func_code = *(reinterpret_cast<uint8_t *>(pbuff + 1));
      uint16_t id = *(reinterpret_cast<uint16_t *>(pbuff + 2));
      uint16_t len = *(reinterpret_cast<uint16_t *>(pbuff + 4));
      data = new uint8_t[len];
      std::memcpy(data, reinterpret_cast<uint16_t *>(pbuff + 6), len);
      uint16_t crc = *(reinterpret_cast<uint16_t *>(pbuff + 6 + len));
      auto verify = custom_serial::Verify_CRC16_Check_Sum(data, len);

      auto s = fmt::format(
          "fun=0x{:02X}, id=0x{:04X}, len={}, crc=0x{:04X}|0x{:04X};",
          func_code, id, len, crc, verify);
      RCLCPP_DEBUG(this->get_logger(), "%s; data: %s", s.c_str(), data);
      if (len == 0 || crc == verify) {
        // Pulisher msg
        custom_serial_interfaces::msg::Receive receive_msg;
        receive_msg.header.stamp = this->now();
        receive_msg.func_code = func_code;
        receive_msg.id = id;
        receive_msg.len = len;
        receive_msg.data.resize(len);
        std::memcpy(receive_msg.data.data(), data, len);
        m_receive_pub->publish(receive_msg);
      } else {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            fmt::format(
                "id: {:02X}; fun_code: {:04X}; crc failed: {:04X}|{:04X}",
                func_code, id, crc, verify));
      }
      delete[] data;
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         fmt::format("Error header: {:04X}", buff[0]));
    }
  }
}

void SerialDriverNode::sendData(uint8_t func_code, uint16_t id, uint16_t len,
                                const uint8_t *data) {
  std::vector<uint8_t> buff(len + 8);
  uint8_t *pbuff = buff.data();

  *(reinterpret_cast<uint8_t *>(pbuff)) = 0x5A;
  *(reinterpret_cast<uint8_t *>(pbuff + 1)) = func_code;
  *(reinterpret_cast<uint16_t *>(pbuff + 2)) = id;
  *(reinterpret_cast<uint16_t *>(pbuff + 4)) = len;
  std::memcpy(pbuff + 6, data, len);
  auto crc =
      custom_serial::Verify_CRC16_Check_Sum(const_cast<uint8_t *>(data), len);
  *(reinterpret_cast<uint16_t *>(pbuff + 6 + len)) = crc;

  try {
    m_driver->port()->send(buff);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "serial port send failed!");
    reopen();
  }
}
} // namespace custom_serial

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(custom_serial::SerialDriverNode)
