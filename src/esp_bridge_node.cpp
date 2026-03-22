#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class EspBridgeNode : public rclcpp::Node {
public:
  EspBridgeNode() : Node("esp_bridge_node") {
    declareParameters();
    loadParameters();
    openSerialPort();
    createPublishers();
    createCmdVelSubscription();
    createReadTimer();
    RCLCPP_INFO(this->get_logger(), "EspBridgeNode iniciado");
  }

private:
  // -------------------------------------------------------------------------
  // Inicialización
  // -------------------------------------------------------------------------

  void declareParameters() {
    this->declare_parameter<std::string>("port", kDefaultPort);
    this->declare_parameter<int>("baudrate", kDefaultBaudrate);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
  }

  void loadParameters() {
    port_ = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
  }

  void openSerialPort() {
    try {
      serial_port_.Open(port_);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      RCLCPP_INFO(this->get_logger(), "Puerto serial abierto: %s @ %d",
                  port_.c_str(), baudrate_);
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto: %s",
                   port_.c_str());
    }
  }

  void createPublishers() {
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/odom", kQueueSize);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", kQueueSize);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void createCmdVelSubscription() {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", kQueueSize,
        std::bind(&EspBridgeNode::handleCmdVel, this, _1));
  }

  void createReadTimer() {
    read_timer_ = this->create_wall_timer(
        5ms, std::bind(&EspBridgeNode::readSerial, this));
  }

  // -------------------------------------------------------------------------
  // Lectura serial — acumula bytes hasta '\n', luego parsea
  // -------------------------------------------------------------------------

  void readSerial() {
    if (!serial_port_.IsOpen())
      return;

    try {
      while (serial_port_.IsDataAvailable()) {
        uint8_t byte;
        serial_port_.ReadByte(reinterpret_cast<char &>(byte), 0);

        if (byte == '\n') {
          parseLine(rx_line_);
          rx_line_.clear();
        } else {
          rx_line_ += static_cast<char>(byte);
          if (rx_line_.size() > 256)
            rx_line_.clear();
        }
      }
    } catch (...) {
    }
  }

  void parseLine(const std::string &line) {
    double x, y, theta, vl, vr, pl, pr;
    if (std::sscanf(line.c_str(), "ODOM %lf %lf %lf %lf %lf %lf %lf", &x, &y,
                    &theta, &vl, &vr, &pl, &pr) == 7) {
      publishOdom(x, y, theta, vl, vr);
      publishJointStates(pl, pr, vl, vr);
    }
  }

  // -------------------------------------------------------------------------
  // Publicadores ROS2
  // -------------------------------------------------------------------------

  void publishOdom(double x_uwb, double y_uwb, double theta, double vl,
                   double vr) {
    auto now = this->get_clock()->now();

    // ── Offset UWB → base_footprint ────────────────────────────────────────
    // UWB_joint está en xyz=(-0.22, 0, ...) respecto a body_link.
    // base_footprint está +0.22 m adelante del UWB en el frame del robot.
    constexpr double dx_uwb = 0.22;
    constexpr double dy_uwb = 0.0;

    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);

    const double x = x_uwb + cos_t * dx_uwb - sin_t * dy_uwb;
    const double y = y_uwb + sin_t * dx_uwb + cos_t * dy_uwb;

    // ── Odometría ───────────────────────────────────────────────────────────
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    constexpr double r = 0.31 / 2.0;
    constexpr double b = 0.37 / 2.0;
    odom.twist.twist.linear.x = r * (vr + vl) / 2.0;
    odom.twist.twist.angular.z = r * (vr - vl) / (2.0 * b);

    odom_pub_->publish(odom);

    // ── TF odom → base_footprint ────────────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  void publishJointStates(double pl, double pr, double vl, double vr) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->get_clock()->now();
    js.name = {"wheel2_joint", "wheel1_joint"};
    js.position = {pl, pr};
    js.velocity = {vl, vr};
    js.effort = {0.0, 0.0};
    joint_pub_->publish(js);
  }

  // -------------------------------------------------------------------------
  // /cmd_vel → serial
  // -------------------------------------------------------------------------

  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!serial_port_.IsOpen())
      return;

    const float linear = msg->linear.x;
    const float angular = msg->angular.z;

    std::ostringstream ss;
    if (std::abs(linear) < 1e-4f && std::abs(angular) < 1e-4f) {
      ss << "stop\n";
    } else {
      ss << std::fixed << std::setprecision(kDecimalPrecision);
      ss << "twist: " << linear << " " << angular << "\n";
    }

    serial_port_.Write(ss.str());
  }

  // -------------------------------------------------------------------------
  // Constantes
  // -------------------------------------------------------------------------

  static constexpr const char *kDefaultPort = "/dev/ttyUSB0";
  static constexpr int kDefaultBaudrate = 115200;
  static constexpr int kQueueSize = 10;
  static constexpr int kDecimalPrecision = 3;

  // -------------------------------------------------------------------------
  // Miembros
  // -------------------------------------------------------------------------

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  LibSerial::SerialPort serial_port_;
  std::string rx_line_;

  std::string port_;
  int baudrate_;
  std::string odom_frame_;
  std::string base_frame_;
};

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EspBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
