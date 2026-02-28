#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

using std::placeholders::_1;

class EspBridgeNode : public rclcpp::Node {
public:
  EspBridgeNode() : Node("esp_bridge_node") {
    declareParameters();
    loadParameters();
    openSerialPort();
    createCmdVelSubscription();
  }

private:
  // --------------------------------------------------------------------------
  // Initialization
  // --------------------------------------------------------------------------

  void declareParameters() {
    this->declare_parameter<std::string>("port", kDefaultPort);
    this->declare_parameter<int>("baudrate", kDefaultBaudrate);
  }

  void loadParameters() {
    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
  }

  void openSerialPort() {
    try {
      serial_port_.Open(port_);
      configureSerialPort();
      RCLCPP_INFO(this->get_logger(), "Serial port opened on %s at %d baud",
                  port_.c_str(), baudrate_);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                   port_.c_str());
    }
  }

  void configureSerialPort() {
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);

    if (baudrate_ == 115200) {
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unsupported baudrate %d. Falling back to 115200.",
                  baudrate_);
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
  }

  void createCmdVelSubscription() {
    cmd_vel_subscription_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", kQueueSize,
            std::bind(&EspBridgeNode::handleCmdVel, this, _1));
  }

  // --------------------------------------------------------------------------
  // Callback
  // --------------------------------------------------------------------------

  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!serial_port_.IsOpen()) {
      return;
    }

    const float linear = msg->linear.x;
    const float angular = msg->angular.z;

    serial_port_.Write(formatTwistCommand(linear, angular));
  }

  // --------------------------------------------------------------------------
  // Formatting
  // --------------------------------------------------------------------------

  std::string formatTwistCommand(float linear, float angular) const {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(kDecimalPrecision);
    stream << "twist: " << linear << " " << angular << "\n";
    return stream.str();
  }

  // --------------------------------------------------------------------------
  // Constants
  // --------------------------------------------------------------------------

  static constexpr const char *kDefaultPort = "/dev/ttyUSB0";
  static constexpr int kDefaultBaudrate = 115200;
  static constexpr int kQueueSize = 10;
  static constexpr int kDecimalPrecision = 3;

  // --------------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------------

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  LibSerial::SerialPort serial_port_;

  std::string port_;
  int baudrate_;
};

// --------------------------------------------------------------------------
// Main
// --------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EspBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
