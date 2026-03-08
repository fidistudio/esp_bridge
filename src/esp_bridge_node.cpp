#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

// TelemetryFrame debe ser accesible desde el paquete ROS2.
// Copia TelemetryFrame.h al directorio include/ de tu paquete ROS2.
#include "esp_bridge/TelemetryFrame.h"

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

  // Timer a 200 Hz — lee el puerto sin bloquear; los frames llegan a 20 Hz
  void createReadTimer() {
    read_timer_ = this->create_wall_timer(
        5ms, std::bind(&EspBridgeNode::readSerial, this));
  }

  // -------------------------------------------------------------------------
  // Lectura serial y sincronización de frame binario
  // -------------------------------------------------------------------------

  void readSerial() {
    if (!serial_port_.IsOpen())
      return;

    // Drenar bytes disponibles al buffer interno
    try {
      while (serial_port_.IsDataAvailable()) {
        uint8_t byte;
        serial_port_.ReadByte(reinterpret_cast<char &>(byte), 0);
        rx_buf_.push_back(byte);
      }
    } catch (...) {
      return;
    }

    // Intentar parsear frames completos
    while (rx_buf_.size() >= TELEMETRY_FRAME_SIZE) {
      // Buscar magic 0xAA55 — re-sincronización automática
      if (rx_buf_[0] != 0xAA || rx_buf_[1] != 0x55) {
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }

      // Verificar end marker
      if (rx_buf_[TELEMETRY_FRAME_SIZE - 1] != TELEMETRY_END_MARKER) {
        // Frame corrupto: descartar el magic y seguir buscando
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 2);
        continue;
      }

      // Frame completo y válido → deserializar
      TelemetryFrame frame;
      std::memcpy(&frame, rx_buf_.data(), sizeof(TelemetryFrame));
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + TELEMETRY_FRAME_SIZE);

      publishOdom(frame.data);
      publishJointStates(frame.data);
    }

    // Evitar que el buffer crezca indefinidamente si hay ruido
    if (rx_buf_.size() > kMaxBufSize)
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - TELEMETRY_FRAME_SIZE);
  }

  // -------------------------------------------------------------------------
  // Publicadores ROS2
  // -------------------------------------------------------------------------

  void publishOdom(const TelemetryPayload &d) {
    auto now = this->get_clock()->now();

    // ---- Odometry message ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // Pose
    odom.pose.pose.position.x = d.x;
    odom.pose.pose.position.y = d.y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, d.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Twist (velocidad del robot en frame base_link)
    // v = (r_R·ω_R + r_L·ω_L) / 2
    // ω = (r_R·ω_R - r_L·ω_L) / (2b)
    constexpr double r = 0.31 / 2.0; // radio de rueda [m]
    constexpr double b = 0.37 / 2.0; // semidistancia entre ruedas [m]
    odom.twist.twist.linear.x = r * (d.vel_right + d.vel_left) / 2.0;
    odom.twist.twist.angular.z = r * (d.vel_right - d.vel_left) / (2.0 * b);

    odom_pub_->publish(odom);

    // ---- TF: odom → base_link ----
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = d.x;
    tf.transform.translation.y = d.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  void publishJointStates(const TelemetryPayload &d) {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->get_clock()->now();

    js.name = {"wheel2_joint", "wheel1_joint"};
    js.position = {d.pos_left, d.pos_right};
    js.velocity = {d.vel_left, d.vel_right};
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
  static constexpr std::size_t kMaxBufSize = 512;

  // -------------------------------------------------------------------------
  // Miembros
  // -------------------------------------------------------------------------

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Timer de lectura serial
  rclcpp::TimerBase::SharedPtr read_timer_;

  // Serial
  LibSerial::SerialPort serial_port_;
  std::vector<uint8_t> rx_buf_;

  // Parámetros
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
