// stdlib
#include <limits>

// ROS2 Core
#include <mitsubishi_robot_interfaces/srv/detail/svg_executor__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mitsubishi_robot_interfaces/srv/svg_executor.hpp>

// MoveIt2
#include <moveit_ros_planning_interface/moveit/move_group_interface/move_group_interface.h>

// Math
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/tf2_ros/buffer.h>
#include <tf2_ros/tf2_ros/transform_listener.h>

class SvgPathExecutor : public rclcpp::Node {
public:
  // ctor, dtor, cctor, operator=
  SvgPathExecutor()
    : Node("svg_path_executor"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    // Parameters
    declare_parameter<std::string>("move_group", "rv_6sdl_s15");
    this->move_group_name_ = get_parameter("move_group").as_string();

    // MoveGroupInterface
    this->move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),
            move_group_name_
            );
    this->move_group_->setPlanningTime(5.0);
    this->move_group_->setMaxVelocityScalingFactor(0.2);
    this->move_group_->setMaxAccelerationScalingFactor(0.1);

    // Topic subscription
    this->sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/svg_path", 10,
        std::bind(
          &SvgPathExecutor::subCallback,
          this,
          std::placeholders::_1
          )
        );

    // Service to start drawing
    this->service_ = this->create_service<mitsubishi_robot_interfaces::srv::SvgExecutor>(
        "execute_svg",
        std::bind(&SvgPathExecutor::srvCallback,
          this,
          std::placeholders::_1,
          std::placeholders::_2
          )
        );

    RCLCPP_INFO(this->get_logger(), "SVG Path executor is ready");
  }
  ~SvgPathExecutor() = default;
  SvgPathExecutor(SvgPathExecutor &&) = delete;
  SvgPathExecutor &operator=(SvgPathExecutor &&) = delete;
  SvgPathExecutor(const SvgPathExecutor &) = delete;
  SvgPathExecutor &operator=(const SvgPathExecutor &) = delete;

private:
  // Callbacks
  void subCallback(const std_msgs::msg::Float32MultiArray &msg) {
    this->paths_.clear();
    const auto &data = msg.data;
    if (data.size() % 2 != 0) {
      RCLCPP_WARN(this->get_logger(), "Odd number of floats - ignoring path");
      return;
    }

    // Process received paths
    this->paths_.emplace_back();
    std::vector<Eigen::Vector2d> &path = *this->paths_.rbegin();
    for (std::size_t i = 0; i < data.size(); ++i) {
      const double x = data[i];
      const double y = data[i];

      if (x == std::numeric_limits<double>::signaling_NaN()) {
        this->paths_.emplace_back();
        path = *this->paths_.rbegin();
        continue;
      }
      path.emplace_back(x, y);
    }
    // Ending NaN will create an additional empty path, remove it
    this->paths_.pop_back();

    RCLCPP_INFO(this->get_logger(),
        "Received %lu XY paths (%lu points)",
        this->paths_.size(),
        data.size() - 2 * this->paths_.size()
        );
  }

  void srvCallback(
      const mitsubishi_robot_interfaces::srv::SvgExecutor::Request::SharedPtr request,
      mitsubishi_robot_interfaces::srv::SvgExecutor::Response::SharedPtr response
      )
  {
    RCLCPP_INFO(this->get_logger(), "Received request '%s'", request->action.c_str());
    if(request->action == "trace_bounding_box"){
      response->result = this->traceBoundingBox();
    } else if (request->action == "draw_svg"){
      response->result = this->drawSVG();
    } else {
      response->result = false;;
    }
  }

  // Actions
  bool traceBoundingBox(){
    return true;
  }

  bool drawSVG(){
    return true;
  }

  // ROS2 Core
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Service<mitsubishi_robot_interfaces::srv::SvgExecutor>::SharedPtr service_;

  // MoveIt2
  std::string move_group_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // Data
  std::vector<std::vector<Eigen::Vector2d>> paths_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SvgPathExecutor>();

  while (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  return 0;
}
