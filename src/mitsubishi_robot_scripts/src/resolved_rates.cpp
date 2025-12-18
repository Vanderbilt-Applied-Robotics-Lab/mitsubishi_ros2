#include <Eigen/Dense>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/joint.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <urdf_model/types.h>

// Type specifications, for simplicity
using Vector3f = Eigen::Vector3f;
using Vector6f = Eigen::Vector<float, 6>;
using Matrix3f = Eigen::Matrix3f;
using Matrix4f = Eigen::Matrix4f;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Jacobian = Matrix6f;
using PosJacobian = Eigen::Matrix<float, 3, 6>;

// A singular frame, I named it TrajectoryPoint since that is the context that
// it is used in
struct TrajectoryPoint {
  Vector3f point;
  Matrix3f rot;
};

// Combines an end-effector velocity and angular velocity into a twist vector
inline Vector6f makeTwist(const Vector3f &v, const Vector3f &omega) {
  return Vector6f{v[0], v[1], v[2], omega[0], omega[1], omega[2]};
}

inline float sgn(const float &val) { return val < 0.0 ? -1.0 : 1.0; }

template <typename T, size_t M, size_t N>
inline Eigen::Matrix<T, N, M> pinv(const Eigen::Matrix<T, M, N> &matrix) {
  using matrix_t = Eigen::Matrix<T, M, N>;
  using pinv_matrix_t = Eigen::Matrix<T, N, M>;
  Eigen::CompleteOrthogonalDecomposition<matrix_t> cod(matrix);
  pinv_matrix_t pinv = cod.pseudoInverse();
  return pinv;
}

inline KDL::JntArray q2jnt(const Vector6f &q, const KDL::Chain &chain) {
  KDL::JntArray jnt_arr(chain.getNrOfJoints());
  size_t joint_num = 0;
  for (size_t i = 0; i < chain.getNrOfJoints(); ++i) {
    const KDL::Joint joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed) {
      jnt_arr(i) = q[joint_num++];
    }
  }
  return jnt_arr;
}

inline Vector6f jnt2q(const KDL::JntArray &jnt_arr, const KDL::Chain &chain) {
  Vector6f q;
  size_t joint_num = 0;
  for (size_t i = 0; i < chain.getNrOfJoints(); ++i) {
    const KDL::Joint joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::Fixed) {
      q[joint_num++] = jnt_arr(i);
    }
  }
  return q;
}

inline int getHomogenousTransform(
    const Vector6f &q, Matrix4f &htf, const KDL::Chain &chain,
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver) {
  KDL::JntArray jnt_arr = q2jnt(q, chain);
  KDL::Frame frame;
  if (fk_solver->JntToCart(jnt_arr, frame) < 0) {
    // error
    return -1;
  }
  for (size_t i = 0; i < (size_t) htf.rows(); ++i) {
    for (size_t j = 0; j < (size_t) htf.cols(); ++j) {
      htf(i, j) = frame(i, j);
    }
  }
  return 0;
}

inline void getRotFromHTF(const Matrix4f &htf, Matrix3f &rot) {
  rot = htf.block<3, 3>(0, 0);
}

inline void getPosFromHTF(const Matrix4f &htf, Vector3f &pos) {
  pos = htf.block<3, 1>(0, 3);
}

inline int getJacobian(const Vector6f &q, Matrix6f &jacobian,
                       const KDL::Chain &chain,
                       std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver) {
  KDL::JntArray jnt_arr = q2jnt(q, chain);
  KDL::Jacobian kdl_jac(chain.getNrOfJoints());
  int solver_result;

  solver_result = jac_solver->JntToJac(jnt_arr, kdl_jac);
  if (solver_result != KDL::SolverI::E_NOERROR) {
    return solver_result;
  }
  if (kdl_jac.rows() != 6 && kdl_jac.columns() != 6) {
    return -2;
  }
  for (size_t i = 0; i < (size_t) jacobian.rows(); ++i) {
    for (size_t j = 0; j < (size_t) jacobian.cols(); ++j) {
      jacobian(i, j) = kdl_jac(i, j);
    }
  }
  return 0;
}

inline float manipulabilityMeasure(const Jacobian &j) {
  return sqrt((j.transpose() * j).determinant());
}

int main(int argc, char **argv) {
  ///////////////
  // Constants //
  ///////////////
  static const std::string move_group = "mitsubishi_robot_arm";
  static const std::string controller = "position_controller";
  static const std::string base_link = "base_link";
  static const std::string end_link = "end_effector";
  static const std::string robot_desc_topic = "/robot_description";
  static const std::string joint_state_topic = "/joint_states";
  static constexpr float loop_frequency = 120.0;
  static constexpr float loop_period = 1 / loop_frequency;

  /////////////////////////
  // ROS2 initialization //
  /////////////////////////
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      std::make_shared<rclcpp::Node>("follow_circle");
  auto command_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + controller + "/commands", 10);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  rclcpp::Rate loop_rate(loop_frequency);
  RCLCPP_INFO(node->get_logger(), "Initialized");

  ///////////////////////
  // Robot Description //
  ///////////////////////
  // See https://arnebaeyens.com/blog/2024/kdl-ros2/ for more info
  std::string robot_desc;
  auto set_robot_desc =
      [&robot_desc](const std_msgs::msg::String &robot_desc_msg) {
        robot_desc = robot_desc_msg.data;
      };
  auto robot_desc_sub = node->create_subscription<std_msgs::msg::String>(
      robot_desc_topic,
      rclcpp::QoS(rclcpp::KeepLast(1))
          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      set_robot_desc);
  while (robot_desc.empty()) {
    rclcpp::spin_some(node);
  }
  RCLCPP_INFO(node->get_logger(), "Received robot_description");
  // Parse URDF
  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_desc)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to parse URDF");
    exit(-1);
  } else {
    RCLCPP_INFO(node->get_logger(), "Successfully parsed URDF");
  }

  ////////////////////////
  // Kinematics solvers //
  ////////////////////////
  // Make solve objects
  KDL::Tree robot_tree;
  KDL::Chain robot_chain;
  kdl_parser::treeFromString(robot_desc, robot_tree);
  robot_tree.getChain(base_link, end_link, robot_chain);
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver =
      std::make_shared<KDL::ChainFkSolverPos_recursive>(robot_chain);
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver =
      std::make_shared<KDL::ChainJntToJacSolver>(robot_chain);
  // Bind jacobian function
  auto getHTF = std::bind(getHomogenousTransform, std::placeholders::_1,
                          std::placeholders::_2, robot_chain, fk_solver);
  // Bind homogenous transform function
  auto getJac = std::bind(getJacobian, std::placeholders::_1,
                          std::placeholders::_2, robot_chain, jac_solver);
  // Get joint limits and map
  std::unordered_map<std::string, size_t> joint_map;
  std::vector<std::string> joint_names;
  std::vector<float> q_min;
  std::vector<float> q_max;
  std::vector<float> q_avg;
  q_min.resize(robot_chain.getNrOfJoints());
  q_max.resize(robot_chain.getNrOfJoints());
  q_avg.resize(robot_chain.getNrOfJoints());
  for (size_t i = 0; i < robot_chain.getNrOfJoints(); ++i) {
    const KDL::Joint &joint = robot_chain.getSegment(i).getJoint();
    const std::string &joint_name = joint.getName();
    joint_map[joint_name] = i;
    urdf::JointConstSharedPtr urdf_joint = urdf_model.getJoint(joint_name);
    if (urdf_joint && urdf_joint->limits) {
      q_min[i] = urdf_joint->limits->lower;
      q_max[i] = urdf_joint->limits->upper;
      q_avg[i] = (q_min[i] + q_max[i]) / 2;
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "Failed to get joint/limits for joint '%s'",
                  joint_name.c_str());
    }
  }

  ////////////////
  // Trajectory //
  ////////////////
  // Settings
  constexpr size_t traj_len = 100;
  constexpr float traj_circle_rad = 0.1;           // [m]
  const Vector3f traj_point_offset{0.6, 0.0, 0.5}; // [m]
  // Generation
  std::array<TrajectoryPoint, traj_len> traj;
  for (size_t i = 0; i < traj_len; ++i) {
    TrajectoryPoint &cur_traj_point = traj[i];
    float theta = 2 * M_PI * ((float)i / (float)traj_len);      // [rad]
    cur_traj_point.point = Vector3f(0, sin(theta), cos(theta)); // [m]
    cur_traj_point.point *= traj_circle_rad;                    // [m]
    cur_traj_point.point += traj_point_offset;                  // [m]
    cur_traj_point.rot << 0.0, 0.0, 1.0,
                          0.0, -1.0, 0.0,
                          1.0, 0.0, 0.0;
  }
  // Publish path
  nav_msgs::msg::Path path_msg;
  for (size_t i = 0; i < traj_len; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = traj[i].point[0];
    pose.pose.position.y = traj[i].point[1];
    pose.pose.position.z = traj[i].point[2];
    path_msg.poses.push_back(pose);
  }
  path_msg.header.frame_id = "world";
  path_pub->publish(path_msg);

  ////////////////////
  // Resolved Rates //
  ////////////////////
  // Constants
  constexpr float omega_max = 1;         // [rad/s]
  constexpr float omega_min = 0.01;        // [rad/s]
  constexpr float epsilon_omega = 0.1;     // [rad]
  constexpr float lambda_omega = 5;        // [1]
  constexpr float v_max = 1;             // [m/s]
  constexpr float v_min = 0.01;            // [m/s]
  constexpr float epsilon_p = 0.001;       // [m]
  constexpr float lambda_p = 5;            // [1]
  constexpr bool repeat_trajectory = true; // [1]
  // Variables
  /// The naming here should mostly align with the 'Resolved Rates handout' from
  /// class/BrightSpace
  Matrix4f htf;                   // [-]
  Vector6f q_cur;                 // [rad]
  Vector3f p_cur;                 // [m]
  Vector3f p_des = traj[0].point; // [m]
  Vector3f p_e;                   // [m]
  Vector3f v_des;                 // [m/s]
  Vector3f p_n;                   // [m]
  float delta_p;                  // [m]
  float v;                        // [m/s]
  Matrix3f R_cur;                 // [1]
  Matrix3f R_des = traj[0].rot;   // [1]
  Matrix3f R_e;                   // [1]
  Eigen::AngleAxisf R_axis_angle; // [-]
  Vector3f omega_des;             // [rad/s]
  float omega;                    // [rad/s]
  float delta_omega;              // [rad]
  Vector6f twist_des;             // [1]
  Vector6f qd_des;                // [rad/s]
  size_t traj_index = 0;          // [1]
  Vector6f w_elements;            // [1]
  Matrix6f W;                     // [1]
  Jacobian J;                     // [1]
  PosJacobian J_pos;              // [1]
  // Joint state subscriber
  /// Declared after since it uses 'q_cur'
  bool new_q = false;
  auto updateJointStates =
      [&q_cur, &joint_map,
       &new_q](const sensor_msgs::msg::JointState &joint_states) {
        for (size_t i = 0; i < joint_states.position.size(); ++i) {
          new_q = true;
          q_cur[joint_map[joint_states.name[i]]] = joint_states.position[i];
        }
      };
  auto joint_state_sub =
      node->create_subscription<sensor_msgs::msg::JointState>(
          joint_state_topic, 1, updateJointStates);
  while (!new_q) {
    rclcpp::spin_some(node);
    usleep(10);
  }
  // Loop
  RCLCPP_INFO(node->get_logger(), "Starting resolved rates loop");
  do {

    /// Get current joint values by spinning the node; that is, processing
    /// callbacks for subscribers, notably the joint state subscriber. I could
    /// implement a check here for whether the var has been updated, but as it
    /// stands there isn't one
    rclcpp::spin_some(node);

    if (!new_q) {
      continue;
    }

    // Check joint limits
    for (size_t i = 0; i < (size_t) q_cur.size(); ++i) {
      if (q_cur[i] < q_min[i] || q_max[i] < q_cur[i]) {
        RCLCPP_FATAL(node->get_logger(),
                     "Joint %lu exceeded limit %f < %f < %f", i, q_min[i],
                     q_cur[i], q_max[i]);
        rclcpp::shutdown();
        continue;
      } else if (q_cur[i] < q_min[i] + 0.1 || q_max[i] - 0.1 < q_cur[i]) {
        RCLCPP_WARN(node->get_logger(), "Joint %lu near limit %f < %f < %f", i,
                    q_min[i], q_cur[i], q_max[i]);
      }
    }

    /// Use the updated q_cur
    if (getHTF(q_cur, htf) < 0) {
      RCLCPP_FATAL(node->get_logger(),
                   "Error while calculating homogenous transform");
    }
    getPosFromHTF(htf, p_cur);
    getRotFromHTF(htf, R_cur);

    // Calculate posistional error
    p_e = p_des - p_cur;
    delta_p = p_e.norm();

    // Calculate rotational error
    R_e = R_des * R_cur.transpose();
    R_axis_angle.fromRotationMatrix(R_e);
    delta_omega = R_axis_angle.angle();

    /// Update p_des and R_des based on proximity to next point This allows for
    /// smoother following of curved paths. I could also do an interpolation
    /// between all the points to remove sharp corners, but this suffices for
    /// now
    if (delta_p < lambda_p * epsilon_p &&
        delta_omega < lambda_omega * epsilon_omega) {
      if (traj_index == traj_len - 1 && repeat_trajectory) {
        traj_index = 0;
        p_des = traj[traj_index].point;
        R_des = traj[traj_index].rot;
        continue;
      } else {
        ++traj_index;
        p_des = traj[traj_index].point;
        R_des = traj[traj_index].rot;
        continue;
      }
    }
    if (traj_index != traj_len - 1 && delta_p < lambda_p * epsilon_p &&
        delta_omega < lambda_omega * epsilon_omega) {
      /// This iteration would slow the robot's velocity, so instead of slowing
      /// to the desired point, we will just skip to the next one in the
      /// trajectory. We do not do this for the final point in the trajectory
      /// unless repeat_trajectory is true.
      ++traj_index;
      p_des = traj[traj_index].point;
      R_des = traj[traj_index].rot;
      continue;
    }

    /// Calculate desired positional velocity magnitude
    if (delta_p > lambda_p * epsilon_p) {
      /// Not within 'slow down' error
      v = v_max;
    } else {
      /// Within 'slow down' error
      v = v_min + ((v_max - v_min) * (delta_p - epsilon_p)) /
                      (epsilon_p * (lambda_p - 1));
    }
    /// Calculate desired positional velocity direction
    p_n = p_e.normalized();
    /// Calculate desired positional velocity
    v_des = v * p_n;

    /// Calculate desired angular velocity speed
    if (delta_omega > lambda_omega * epsilon_omega) {
      /// Not within 'slow down' error
      omega = omega_max;
    } else {
      /// Within 'slow down' error
      omega = omega_min +
              ((omega_max - omega_min) * (delta_omega - epsilon_omega)) /
                  (epsilon_omega * (lambda_omega - 1));
    }
    /// Calculate desired angular velocity
    omega_des = omega * R_axis_angle.axis();

    /// Calculate desired twist
    twist_des = makeTwist(v_des, omega_des);

    //////////////////////
    // Solve for qd_des //
    //////////////////////
    // Jacobian
    int jacobian_error_code;
    if ((jacobian_error_code = getJac(q_cur, J)) != KDL::SolverI::E_NOERROR) {
      RCLCPP_WARN(node->get_logger(), "Error %d while computing jacobian",
                  jacobian_error_code);
    }
    float manipulability = manipulabilityMeasure(J);
    if (manipulability < 0.001) {
      RCLCPP_WARN(node->get_logger(), "Manipulability measure %f < 0.001",
                  manipulability);
    }
    qd_des = pinv<float, 6, 6>(J) * twist_des;
    
    // Debug
    // std::stringstream ss;
    // ss << qd_des.transpose();
    // RCLCPP_INFO(node->get_logger(), "qd_des = %s", ss.str().c_str());

    // Manually integrate to get new position command
    q_cur += qd_des * loop_period;

    // Send position command
    std_msgs::msg::Float64MultiArray q_cur_msg;
    q_cur_msg.data.resize(robot_chain.getNrOfJoints());
    for (size_t i = 0; i < (size_t) q_cur.rows(); ++i) {
      q_cur_msg.data[i] = q_cur[i];
    }
    command_pub->publish(q_cur_msg);

    // Maintain loop frequency
    loop_rate.sleep();
  } while ((delta_p > epsilon_p || delta_omega > epsilon_omega) &&
           rclcpp::ok());
  RCLCPP_INFO(node->get_logger(), "Shutting down");

  rclcpp::shutdown();
}
