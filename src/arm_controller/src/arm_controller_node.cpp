#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include "arm_controller/kinematics_solver.hpp"
#include "arm_controller/cubic_spline.hpp"

#include <fstream>
#include <sstream>
#include <mutex>

namespace arm_controller
{

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode();

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void trajectoryTimerCallback();
  void publishJointCommand(const std::vector<double> & positions);
  std::string loadUrdfFromFile(const std::string & path);

  // Publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;

  // Kinematics and trajectory
  KinematicsSolver kinematics_;
  CubicSpline spline_;

  // State
  std::vector<double> current_positions_;
  std::mutex state_mutex_;
  bool trajectory_active_;
  double trajectory_start_time_;
  std::string active_arm_;

  // Joint names
  std::vector<std::string> all_joint_names_;
};

}  // namespace arm_controller

namespace arm_controller
{

ArmControllerNode::ArmControllerNode()
: Node("arm_controller_node"),
  trajectory_active_(false),
  trajectory_start_time_(0.0),
  active_arm_("arm_A")
{
  // Declare parameters
  this->declare_parameter<std::string>("urdf_path", "");
  this->declare_parameter<double>("control_rate", 100.0);
  this->declare_parameter<double>("trajectory_duration", 3.0);

  std::string urdf_path = this->get_parameter("urdf_path").as_string();
  double control_rate = this->get_parameter("control_rate").as_double();

  // Load URDF and initialize kinematics
  std::string urdf_string = loadUrdfFromFile(urdf_path);
  if (!kinematics_.initialize(urdf_string)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize kinematics");
  }

  // Initialize joint names for all arms
  all_joint_names_ = {
    "joint_platform_D1",
    "joint_platform_A1", "joint_platform_A2", "joint_platform_A3",
    "joint_platform_A4", "joint_platform_A5", "joint_platform_A6",
    "joint_platform_B1", "joint_platform_B2", "joint_platform_B3",
    "joint_platform_B4", "joint_platform_B5", "joint_platform_B6",
    "joint_platform_S1", "joint_platform_S2", "joint_platform_S3",
    "joint_platform_S4", "joint_platform_S5", "joint_platform_S6"
  };

  current_positions_.resize(all_joint_names_.size(), 0.0);

  // Create publishers and subscribers
  joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/robot/joint_command", 10);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/robot/joint_states", 10,
    std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));

  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", 10,
    std::bind(&ArmControllerNode::targetPoseCallback, this, std::placeholders::_1));

  // Create trajectory timer
  auto period = std::chrono::duration<double>(1.0 / control_rate);
  trajectory_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ArmControllerNode::trajectoryTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Arm controller initialized");
}

std::string ArmControllerNode::loadUrdfFromFile(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open URDF file: %s", path.c_str());
    return "";
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void ArmControllerNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t j = 0; j < all_joint_names_.size(); ++j) {
      if (msg->name[i] == all_joint_names_[j] && i < msg->position.size()) {
        current_positions_[j] = msg->position[i];
        break;
      }
    }
  }
}

void ArmControllerNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Extract arm name from frame_id (e.g., "arm_A", "arm_B", "arm_S")
  active_arm_ = msg->header.frame_id;
  if (active_arm_.empty()) {
    active_arm_ = "arm_A";
  }

  // Convert pose to KDL frame
  KDL::Frame target_frame;
  target_frame.p = KDL::Vector(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);
  target_frame.M = KDL::Rotation::Quaternion(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);

  // Get current arm joint positions as seed
  std::vector<double> seed(6, 0.0);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    int offset = 1;  // Skip D1
    if (active_arm_ == "arm_B") offset = 7;
    else if (active_arm_ == "arm_S") offset = 13;
    for (int i = 0; i < 6; ++i) {
      seed[i] = current_positions_[offset + i];
    }
  }

  // Solve IK
  std::vector<double> solution;
  if (!kinematics_.solveIK(active_arm_, target_frame, seed, solution)) {
    RCLCPP_WARN(this->get_logger(), "IK failed for %s", active_arm_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "IK solved for %s", active_arm_.c_str());

  // Create cubic spline trajectory
  double duration = this->get_parameter("trajectory_duration").as_double();
  std::vector<double> times = {0.0, duration};
  std::vector<std::vector<double>> waypoints = {seed, solution};

  spline_.setWaypoints(times, waypoints);
  trajectory_start_time_ = this->now().seconds();
  trajectory_active_ = true;
}

void ArmControllerNode::trajectoryTimerCallback()
{
  if (!trajectory_active_) {
    return;
  }

  double elapsed = this->now().seconds() - trajectory_start_time_;
  double duration = spline_.getDuration();

  if (elapsed >= duration) {
    trajectory_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Trajectory completed");
    return;
  }

  std::vector<double> position, velocity;
  if (spline_.sample(elapsed, position, velocity)) {
    publishJointCommand(position);
  }
}

void ArmControllerNode::publishJointCommand(const std::vector<double> & positions)
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  // Determine joint names based on active arm
  std::vector<std::string> arm_joints;
  if (active_arm_ == "arm_A") {
    arm_joints = {
      "joint_platform_A1", "joint_platform_A2", "joint_platform_A3",
      "joint_platform_A4", "joint_platform_A5", "joint_platform_A6"
    };
  } else if (active_arm_ == "arm_B") {
    arm_joints = {
      "joint_platform_B1", "joint_platform_B2", "joint_platform_B3",
      "joint_platform_B4", "joint_platform_B5", "joint_platform_B6"
    };
  } else {
    arm_joints = {
      "joint_platform_S1", "joint_platform_S2", "joint_platform_S3",
      "joint_platform_S4", "joint_platform_S5", "joint_platform_S6"
    };
  }

  msg.name = arm_joints;
  msg.position = positions;

  joint_cmd_pub_->publish(msg);
}

}  // namespace arm_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_controller::ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
