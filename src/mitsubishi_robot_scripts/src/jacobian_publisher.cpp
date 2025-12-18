#include <Eigen/Dense>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

int counter() {
  static int i = 0;
  return i++;
}

class JacobianPublisher : public rclcpp::Node {
public:
  ///////////////////////////////////////////////////////////////////////
  // Constructor, copy-constructor, assignment operator, deconstructor //
  ///////////////////////////////////////////////////////////////////////
  JacobianPublisher() : rclcpp::Node("jacobian_publisher") {
    // Initialize kinematics solvers
    const std::string urdf = this->getURDF();
    int init_kin_result = this->initKinematics(urdf);
    if (init_kin_result < 0) {
      RCLCPP_FATAL(this->get_logger(), "Could not initialize kinematics");
      rclcpp::shutdown();
    }

    // Init data structure
    this->initData();

    // Create JointState subscriber
    auto joint_state_callback = std::bind(
        &JacobianPublisher::joint_state_callback, this, std::placeholders::_1);
    this->joint_state_subscriber =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1, joint_state_callback);
    RCLCPP_INFO(this->get_logger(), "Created JointState subscriber");

    // Create Jacobian publisher
    this->jacobian_publisher =
        this->create_publisher<std_msgs::msg::Float32MultiArray>("jacobian", 1);
    RCLCPP_INFO(this->get_logger(), "Created Jacobian publisher");
    RCLCPP_INFO(this->get_logger(), "Initialization successful");
  }
  JacobianPublisher(const JacobianPublisher &) = delete;
  JacobianPublisher &operator=(const JacobianPublisher &) = delete;
  ~JacobianPublisher() = default;

private:
  ////////////////////
  // Initialization //
  ////////////////////
  std::string getURDF() {
    std::string urdf;
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/robot_state_publisher");
    while (!param_client->wait_for_service()) {
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(this->get_logger(),
                     "Interrupted while waiting for parameter service");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(),
                  "Parameter service unavailable, trying again...");
    }

    auto params = param_client->get_parameters({"robot_description"});
    for (const auto &param : params) {
      if (param.get_name() == "robot_description") {
        urdf = param.value_to_string();
        break;
      }
    }
    return urdf;
  }

  int initKinematics(const std::string &urdf) {
    if (urdf.empty()) {
      return -1;
    }
    kdl_parser::treeFromString(urdf, this->kdl_tree);
    this->kdl_tree.getChain("base_link", "end_effector", this->kdl_chain);
    this->jacobian_solver =
        std::make_shared<KDL::ChainJntToJacSolver>(this->kdl_chain);
    for (size_t i = 0; i < this->kdl_chain.getNrOfJoints(); ++i) {
      const KDL::Joint &joint = this->kdl_chain.getSegment(i).getJoint();
      this->joint_name_map[joint.getName()] = i;
    }
    return 0;
  }

  void initData() {
    this->joint_arr.resize(this->kdl_chain.getNrOfJoints());
    this->jacobian.resize(this->kdl_chain.getNrOfJoints());
    this->jacobian_msg.layout.dim.resize(2);
    this->jacobian_msg.layout.dim[0].label = "row";
    this->jacobian_msg.layout.dim[0].size = jacobian.rows();
    this->jacobian_msg.layout.dim[0].stride = jacobian.rows() * jacobian.columns();
    this->jacobian_msg.layout.dim[1].label = "col";
    this->jacobian_msg.layout.dim[1].size = jacobian.columns();
    this->jacobian_msg.layout.dim[1].stride = jacobian.columns();
    this->jacobian_msg.layout.data_offset = 0;
    this->jacobian_msg.data.resize(jacobian.rows() * jacobian.columns());
  }

  //////////////////////////
  // Joint state callback //
  //////////////////////////
  void joint_state_callback(const sensor_msgs::msg::JointState &msg) {
    // Extract joint states into KDL::JntArray
    for (size_t i = 0; i < msg.position.size(); ++i) {
      size_t joint_arr_i = this->joint_name_map.at(msg.name[i]);
      this->joint_arr(joint_arr_i) = msg.position[i];
    }

    // Solve for Jacobian
    int jacobian_sol_error_code =
        this->jacobian_solver->JntToJac(this->joint_arr, this->jacobian);
    if (jacobian_sol_error_code != KDL::SolverI::E_NOERROR) {
      RCLCPP_ERROR(this->get_logger(), "Error (%d) when solving for Jacobian",
                   jacobian_sol_error_code);
      return;
    }

    // Put Jacobian into Float32MultiArray
    for (size_t i = 0; i < this->jacobian.rows(); ++i) {
      for (size_t j = 0; j < this->jacobian.columns(); ++j) {
        this->jacobian_msg
            .data[i * this->jacobian_msg.layout.dim[1].stride + j] =
            jacobian(i, j);
      }
    }

    // Publish jacobian
    this->jacobian_publisher->publish(this->jacobian_msg);
  }

  //////////
  // Data //
  //////////
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  KDL::JntArray joint_arr;
  KDL::Jacobian jacobian;
  std_msgs::msg::Float32MultiArray jacobian_msg;
  std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
  std::unordered_map<std::string, size_t> joint_name_map;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr jacobian_publisher;
};

int main(int argc, char **argv) {
  //////////////////////////
  // Initialize ROS2 Node //
  //////////////////////////
  rclcpp::init(argc, argv);
  std::shared_ptr<JacobianPublisher> node =
      std::make_shared<JacobianPublisher>();

  ///////////////
  // Spin loop //
  ///////////////
  RCLCPP_INFO(node->get_logger(), "Entering main loop");
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(), "Shutting down");
  rclcpp::shutdown();
  return 0;
}
