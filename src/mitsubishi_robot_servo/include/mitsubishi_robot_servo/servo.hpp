#ifndef SERVO_HPP
#define SERVO_HPP

// ROS2
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// #include <moveit_msgs/srv/servo_command_type.hpp>
#include <mitsubishi_robot_msgs/srv/detail/servo_command_type__struct.hpp>
#include <mitsubishi_robot_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>

// STL
#include <atomic>
#include <mutex>
#include <thread>

class Servo : public rclcpp::Node {
public:
  // ctor, dtor, cctor, =op
  Servo();
  ~Servo() override;
  Servo(const Servo &) = delete;
  Servo &operator=(const Servo &) = delete;

private:
  // Helper functions
  void pauseServo(bool pause);
  void publishTimerCallback();
  void keyLoop();
  void setCommandType(uint8_t type);
  void nextFrame();
  void setTerminalRaw(bool enable);

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  // Clients
  rclcpp::Client<mitsubishi_robot_msgs::srv::ServoCommandType>::SharedPtr input_type_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr servo_enable_client_;

  // Timer callback
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  std::vector<std::string> frames_;
  std::vector<std::string>::iterator frame_;
  uint8_t mode_;
  double linear_step_;
  double angular_step_;
  double joint_step_;
  double publish_rate_;

  // Shared state
  geometry_msgs::msg::TwistStamped twist_cmd_;
  control_msgs::msg::JointJog jj_cmd_;
  std::mutex cmd_mutex_;
  std::atomic_bool shutting_down_;

  // Infrastructure
  std::thread key_thread_;
};

#endif // SERVO_HPP
