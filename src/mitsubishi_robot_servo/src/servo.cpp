#include <future>
#include <mitsubishi_robot_servo/servo.hpp>
#include <chrono>
#include <termios.h>
#include <unistd.h>

Servo::Servo() : Node("servo_keyboard"), shutting_down_(false) {
  // Parameters
  this->frames_ = this->declare_parameter<std::vector<std::string>>("frames", {"base_link", "end_effector"});
  this->linear_step_ = this->declare_parameter<double>("linear_step", 0.01);
  this->angular_step_ = this->declare_parameter<double>("angular_step", 0.05);
  this->joint_step_ = this->declare_parameter<double>("joint_step", 0.05);
  this->publish_rate_ = this->declare_parameter<double>("publish_rate", 150.0);

  // Parameter checking
  if (this->frames_.empty()){
    RCLCPP_FATAL(this->get_logger(), "The 'frames' parameter must contain at least 1 frame");
    this->shutting_down_ = true;
    rclcpp::shutdown();
  }
  if (this->publish_rate_ <= 0.0){
    this->publish_rate_ = 50.0;
  }
  this->linear_step_ = std::abs(this->linear_step_);
  this->angular_step_ = std::abs(this->angular_step_);
  this->joint_step_ = std::abs(this->joint_step_);

  // Print parameters
  std::stringstream ss;
  for(const std::string &frame : this->frames_){
    ss << frame << ' ';
  }
  RCLCPP_INFO(this->get_logger(), "Frames             : [ %s]", ss.str().c_str());
  RCLCPP_INFO(this->get_logger(), "Linear step (m)    : %f", this->linear_step_);
  RCLCPP_INFO(this->get_logger(), "Angular step (rad) : %f", this->angular_step_);
  RCLCPP_INFO(this->get_logger(), "Joint step (rad) : %f", this->joint_step_);
  RCLCPP_INFO(this->get_logger(), "Publish rate (Hz)  : %f", this->publish_rate_);

  // Publishers
  this->twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>( "/servo_node/delta_twist_cmds", 10);
  this->joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>( "/servo_node/delta_joint_cmds", 10);

  // Input type client
  this->input_type_client_ = this->create_client<mitsubishi_robot_msgs::srv::ServoCommandType>( "/servo_node/switch_command_type");
  RCLCPP_INFO(this->get_logger(), "Waiting for '/servo_node/switch_command_type'");
  if (input_type_client_->wait_for_service(std::chrono::seconds(5))) {
    this->mode_ = mitsubishi_robot_msgs::srv::ServoCommandType::Request::JOINT_JOG;
    this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
  } else {
    RCLCPP_FATAL(this->get_logger(), "'%s' service unavailable", this->input_type_client_->get_service_name());
  }

  // Pause servo client
  RCLCPP_INFO(this->get_logger(), "Waiting for '/servo_node/enable_servo'");
  this->servo_enable_client_ = this->create_client<std_srvs::srv::SetBool>( "/servo_node/pause_servo");
  if (this->servo_enable_client_->wait_for_service(std::chrono::seconds(5))) {
    this->pauseServo(false);
  } else {
    RCLCPP_FATAL(this->get_logger(), "'%s' service unavailable", this->servo_enable_client_->get_service_name());
  }

  // Init params
  this->frame_ = this->frames_.begin();
  this->twist_cmd_.header.frame_id = *this->frame_;
  this->jj_cmd_.header.frame_id = "base_link";
  this->jj_cmd_.joint_names = {
    "j1",
    "j2",
    "j3",
    "j4",
    "j5",
    "j6",
  };
  this->jj_cmd_.velocities.resize(6);

  // Periodic callback
  this->publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / this->publish_rate_),
      std::bind(&Servo::publishTimerCallback, this));

  // Input thread
  this->key_thread_ = std::thread([this] { this->keyLoop(); });
}

Servo::~Servo() {
  this->shutting_down_ = true;
  if (this->key_thread_.joinable()) {
    this->key_thread_.join();
  }
  this->pauseServo(true);
}

void Servo::pauseServo(bool pause) {
  if (this->servo_enable_client_->service_is_ready())
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = pause;
    auto future = this->servo_enable_client_->async_send_request(req);
    if(future.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout){
      RCLCPP_WARN(this->get_logger(), "Timed out while attempting to pause Servo");
    }
  }
  RCLCPP_INFO(this->get_logger(), "Servo: %s", pause? "DISABLED": "ENABLED");
}

void Servo::publishTimerCallback() {
  std::lock_guard<std::mutex> lock(this->cmd_mutex_);
  if(this->mode_ == mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST){
    this->twist_cmd_.header.stamp = this->now();
    this->twist_pub_->publish(this->twist_cmd_);
    this->twist_cmd_.twist = geometry_msgs::msg::Twist();
  } else if (this->mode_ == mitsubishi_robot_msgs::srv::ServoCommandType::Request::JOINT_JOG){
    this->jj_cmd_.header.stamp = now();
    this->joint_pub_->publish(this->jj_cmd_);
    std::fill(this->jj_cmd_.displacements.begin(), this->jj_cmd_.displacements.end(), 0);
  } else {
    RCLCPP_FATAL(this->get_logger(), "Command mode in unknown state");
    this->shutting_down_ = true;
    this->pauseServo(true);
    rclcpp::shutdown();
  }
}

void Servo::setCommandType(uint8_t type) {
  if(this->mode_ == type){
    return;
  } else {
    this->twist_cmd_.twist = geometry_msgs::msg::Twist();
    std::fill(this->jj_cmd_.velocities.begin(), this->jj_cmd_.velocities.end(), 0);
  }
  auto req = std::make_shared<mitsubishi_robot_msgs::srv::ServoCommandType::Request>();
  this->mode_ = type;
  req->command_type = type;
  (void)input_type_client_->async_send_request(req);
  RCLCPP_INFO(this->get_logger(), "Command type: %s",
              type == mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST
                  ? "TWIST"
                  : "JOINT JOG");
}

void Servo::nextFrame() {
  if(++this->frame_ == this->frames_.end()){
    this->frame_ = this->frames_.begin();
  }
  this->twist_cmd_.header.frame_id = *this->frame_;
  this->twist_cmd_.twist = geometry_msgs::msg::Twist();
  RCLCPP_INFO(this->get_logger(), "Switched command frame -> %s", this->frame_->c_str());
}

void Servo::setTerminalRaw(bool enable) {
  static struct termios cooked;
  static bool raw = false;
  if (enable && !raw) {
    tcgetattr(STDIN_FILENO, &cooked);
    struct termios t = cooked;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
    raw = true;
  } else if (!enable && raw) {
    tcsetattr(STDIN_FILENO, TCSANOW, &cooked);
    raw = false;
  }
}

void Servo::keyLoop() {
  this->setTerminalRaw(true);
  RCLCPP_INFO(this->get_logger(), "Keyboard control active. Press <C-c> to quit.");
  RCLCPP_INFO(this->get_logger(), "Cartesian Space (Colemak):");
  RCLCPP_INFO(this->get_logger(), "\tr/R ±X          n/N ±Rx");
  RCLCPP_INFO(this->get_logger(), "\ts/S ±Y          e/E ±Ry");
  RCLCPP_INFO(this->get_logger(), "\tt/T ±Z          i/I ±Rz");
  RCLCPP_INFO(this->get_logger(), "\t[/] ±Linear     {/} ±Angular");
  RCLCPP_INFO(this->get_logger(), "\t     Velocity   {/}  Velocity");
  RCLCPP_INFO(this->get_logger(), "Joint Space:");
  RCLCPP_INFO(this->get_logger(), "\t<S-1>/<1> ±J1   <S-4>/<4> ±J4");
  RCLCPP_INFO(this->get_logger(), "\t<S-2>/<2> ±J2   <S-5>/<5> ±J5");
  RCLCPP_INFO(this->get_logger(), "\t<S-3>/<3> ±J3   <S-6>/<6> ±J6");
  while (rclcpp::ok() && !shutting_down_) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) < 1) {
      continue;
    }

    std::lock_guard<std::mutex> lock(this->cmd_mutex_);
    const bool reverse = std::isupper(c);
    const double lin = reverse ? -this->linear_step_ : this->linear_step_;
    const double ang = reverse ? -this->angular_step_ : this->angular_step_;

    switch (std::tolower(c)) {
    // Step adjustments
    case '[':
      this->linear_step_ *= 0.5;
      RCLCPP_INFO(this->get_logger(), "Linear step (m) -> %f", this->linear_step_);
      break;
    case ']':
      this->linear_step_ *= 2.0;
      RCLCPP_INFO(this->get_logger(), "Linear step (m) -> %f", this->linear_step_);
      break;
    case '{':
      this->angular_step_ *= 0.5;
      RCLCPP_INFO(this->get_logger(), "Angular step (rad) -> %f", this->angular_step_);
      break;
    case '}':
      this->angular_step_ *= 2.0;
      RCLCPP_INFO(this->get_logger(), "Angular step (rad) -> %f", this->angular_step_);
      break;

    // Joint
    case '!':
    case '@':
    case '#':
    case '$':
    case '%':
    case '^':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::JOINT_JOG);
      this->jj_cmd_.velocities[c-'!'] -= this->joint_step_;
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::JOINT_JOG);
      this->jj_cmd_.velocities[c-'1'] += this->joint_step_;
      break;

    // Linear
    case 'r':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.linear.x += lin;
      break;
    case 's':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.linear.y += lin;
      break;
    case 't':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.linear.z += lin;
      break;

    // Angular
    case 'n':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.angular.x += ang;
      break;
    case 'e':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.angular.y += ang;
      break;
    case 'i':
      this->setCommandType(mitsubishi_robot_msgs::srv::ServoCommandType::Request::TWIST);
      this->twist_cmd_.twist.angular.z += ang;
      break;

    // Switch frame
    case ' ':
      this->nextFrame();
      break;

    // Pause
    case 'p':
      this->pauseServo(c == 'p');
      break;

    // <C-c>
    case '\x03':
      this->pauseServo(true);
      this->shutting_down_ = true;
      rclcpp::shutdown();
      break;

    default:
      break;
    }
  }
  this->setTerminalRaw(false);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Servo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
