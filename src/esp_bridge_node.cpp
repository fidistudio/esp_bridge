#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using std::placeholders::_1;

class ESPBridgeNode : public rclcpp::Node {
public:
  ESPBridgeNode() : Node("esp_bridge_node") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 115200);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();

    try {
      serial_port_.Open(port_);
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);

      RCLCPP_INFO(this->get_logger(), "Puerto serial abierto en %s",
                  port_.c_str());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto serial");
    }

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ESPBridgeNode::cmdVelCallback, this, _1));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float linear = msg->linear.x;
    float angular = msg->angular.z;

    std::string data =
        std::to_string(linear) + "," + std::to_string(angular) + "\n";

    if (serial_port_.IsOpen()) {
      serial_port_.Write(data);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  LibSerial::SerialPort serial_port_;
  std::string port_;
  int baudrate_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESPBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
