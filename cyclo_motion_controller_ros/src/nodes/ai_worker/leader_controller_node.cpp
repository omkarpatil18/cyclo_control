// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Yeonguk Kim

#include "cyclo_motion_controller_ros/nodes/ai_worker/leader_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cyclo_motion_controller_ros
{
LeaderController::LeaderController()
: Node("leader_controller"),
  right_traj_received_(false),
  left_traj_received_(false),
  lift_joint_received_(false),
  last_right_traj_time_(this->now()),
  last_left_traj_time_(this->now()),
  lift_joint_index_(-1)
{
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Leader Controller - Starting up...");
  RCLCPP_INFO(this->get_logger(), "Node name: %s", this->get_name());
  RCLCPP_INFO(this->get_logger(), "========================================");

  control_frequency_ = this->declare_parameter("control_frequency", 100.0);
  urdf_path_ = this->declare_parameter("urdf_path", std::string(""));
  srdf_path_ = this->declare_parameter("srdf_path", std::string(""));
  joint_states_topic_ = this->declare_parameter("joint_states_topic", std::string("/joint_states"));
  right_traj_topic_ = this->declare_parameter(
            "right_traj_topic",
            std::string("/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory"));
  left_traj_topic_ = this->declare_parameter(
            "left_traj_topic",
            std::string("/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory"));
  reactivate_topic_ = this->declare_parameter("reactivate_topic", std::string("/reactivate"));
  command_timeout_ = this->declare_parameter("command_timeout", 0.1);
  r_goal_pose_topic_ = this->declare_parameter("r_goal_pose_topic", std::string("/r_goal_pose"));
  l_goal_pose_topic_ = this->declare_parameter("l_goal_pose_topic", std::string("/l_goal_pose"));
  r_elbow_pose_topic_ = this->declare_parameter("r_elbow_pose_topic", std::string("/r_elbow_pose"));
  l_elbow_pose_topic_ = this->declare_parameter("l_elbow_pose_topic", std::string("/l_elbow_pose"));
  base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_link"));
  r_gripper_name_ = this->declare_parameter("r_gripper_name", std::string("arm_r_link7"));
  l_gripper_name_ = this->declare_parameter("l_gripper_name", std::string("arm_l_link7"));
  r_elbow_name_ = this->declare_parameter("r_elbow_name", std::string("arm_r_link4"));
  l_elbow_name_ = this->declare_parameter("l_elbow_name", std::string("arm_l_link4"));
  lift_joint_name_ = this->declare_parameter("lift_joint_name", std::string("lift_joint"));
  model_lift_joint_name_ = this->declare_parameter("model_lift_joint_name", std::string("joint"));
  // Motion amplification: the hand goal is scaled about a pivot link (default: the shoulder),
  //   p_goal = p_pivot + motion_scale * (p_hand - p_pivot).   Orientation is passed through.
  // motion_scale = 1.0 reproduces the original 1:1 behaviour.
  motion_scale_ = this->declare_parameter("motion_scale", 1.0);
  r_pivot_name_ = this->declare_parameter("r_motion_scale_pivot_link", std::string("arm_r_link2"));
  l_pivot_name_ = this->declare_parameter("l_motion_scale_pivot_link", std::string("arm_l_link2"));
  if (motion_scale_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "motion_scale=%.3f is invalid; using 1.0", motion_scale_);
    motion_scale_ = 1.0;
  }
  // Frame re-anchoring: the leader and the follower are different bodies (leader shoulders sit
  // 4.25 cm farther from the midline). When enabled, goals are expressed relative to the leader
  // shoulder and re-attached to the FOLLOWER shoulder position (base_link frame, lift = 0):
  //   target = follower_shoulder + motion_scale * (hand - leader_shoulder)
  // Defaults are the FFW SG2 follower shoulders (arm_*_link2 origins from ffw_sg2_follower.urdf).
  // The z of the follower shoulder follows the follower lift joint read from /joint_states.
  remap_to_follower_shoulder_ = this->declare_parameter("remap_to_follower_shoulder", false);
  r_target_pivot_xyz_ = this->declare_parameter(
    "r_target_pivot_xyz", std::vector<double>{-0.0199, -0.2275, 1.4316});
  l_target_pivot_xyz_ = this->declare_parameter(
    "l_target_pivot_xyz", std::vector<double>{-0.0199, 0.2275, 1.4316});
  for (const auto * v : {&r_target_pivot_xyz_, &l_target_pivot_xyz_}) {
    if (v->size() != 3) {
      RCLCPP_ERROR(this->get_logger(),
        "*_target_pivot_xyz must have 3 elements (got %zu); re-anchoring disabled.", v->size());
      remap_to_follower_shoulder_ = false;
    }
  }

  r_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            right_traj_topic_, 10,
            std::bind(&LeaderController::rightTrajectoryCallback, this, std::placeholders::_1));
  l_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            left_traj_topic_, 10,
            std::bind(&LeaderController::leftTrajectoryCallback, this, std::placeholders::_1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 10,
            std::bind(&LeaderController::jointStateCallback, this, std::placeholders::_1));
  reactivate_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            reactivate_topic_, 10,
            std::bind(&LeaderController::reactivateCallback, this, std::placeholders::_1));

  r_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            r_goal_pose_topic_, 10);
  l_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            l_goal_pose_topic_, 10);
  r_elbow_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            r_elbow_pose_topic_, 10);
  l_elbow_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            l_elbow_pose_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Reactivate topic subscribed: %s", reactivate_topic_.c_str());

  try {
    if (urdf_path_.empty()) {
      throw std::runtime_error("URDF path not provided.");
    }
    RCLCPP_INFO(this->get_logger(), "URDF path: %s", urdf_path_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to resolve robot model paths: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  try {
    if (srdf_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "SRDF path not provided. Continuing without SRDF.");
    } else {
      RCLCPP_INFO(this->get_logger(), "SRDF path: %s", srdf_path_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Loading URDF and initializing kinematics solver...");
    kinematics_solver_ =
      std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(urdf_path_,
        srdf_path_);

            // Initialize state variables
    const int dof = kinematics_solver_->getDof();
    q_.setZero(dof);
    qdot_.setZero(dof);
    RCLCPP_INFO(this->get_logger(), "Kinematics solver initialized (DOF: %d)", dof);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize kinematics solver: %s", e.what());
    rclcpp::shutdown();
    return;
  }

        // Initialize joint configuration from URDF
  initializeJointConfig();

  const int timer_period_ms = static_cast<int>(1000.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period_ms),
            std::bind(&LeaderController::controlLoopCallback, this));

  if (!control_timer_) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create control loop timer!");
    rclcpp::shutdown();
    return;
  }

  // The pivot links are needed whenever scaling or re-anchoring is active; validate them once
  // here (a typo must not throw inside the control loop).
  pivot_links_valid_ = true;
  for (const auto & pivot : {r_pivot_name_, l_pivot_name_}) {
    if (!kinematics_solver_->hasLinkFrame(pivot)) {
      RCLCPP_ERROR(this->get_logger(),
        "pivot link '%s' not found in the leader URDF; motion scaling and shoulder re-anchoring "
        "are disabled (goals pass through 1:1).", pivot.c_str());
      pivot_links_valid_ = false;
    }
  }
  if (!pivot_links_valid_) {
    motion_scale_ = 1.0;
    remap_to_follower_shoulder_ = false;
  }
  RCLCPP_INFO(this->get_logger(), "  - motion_scale: %.3f (pivots: %s / %s)",
    motion_scale_, r_pivot_name_.c_str(), l_pivot_name_.c_str());
  if (remap_to_follower_shoulder_) {
    RCLCPP_INFO(this->get_logger(),
      "  - goals re-anchored on follower shoulders R(%.4f, %.4f, %.4f) L(%.4f, %.4f, %.4f) + lift",
      r_target_pivot_xyz_[0], r_target_pivot_xyz_[1], r_target_pivot_xyz_[2],
      l_target_pivot_xyz_[0], l_target_pivot_xyz_[1], l_target_pivot_xyz_[2]);
  } else {
    RCLCPP_INFO(this->get_logger(), "  - goals stay in the leader frame (no re-anchoring)");
  }
  RCLCPP_INFO(this->get_logger(),
            "Leader Controller initialized successfully!");
  RCLCPP_INFO(this->get_logger(),
            "  - Control loop: %.1f Hz (period: %d ms)", control_frequency_, timer_period_ms);
  RCLCPP_INFO(this->get_logger(),
            "  - Subscriptions: joint_states=%s",
            joint_state_sub_ ? "OK" : "FAILED");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Node is ready! Waiting for messages...");
  RCLCPP_WARN(
            this->get_logger(),
            "Control loop is ready. Publish Bool on '%s' to toggle controller output.",
            reactivate_topic_.c_str());
}

LeaderController::~LeaderController()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Leader Controller");
}

void LeaderController::initializeJointConfig()
{
  const auto joint_names = kinematics_solver_->getJointNames();
  model_joint_index_map_.clear();
  for (size_t i = 0; i < joint_names.size(); ++i) {
    model_joint_index_map_[joint_names[i]] = static_cast<int>(i);
  }

  auto it = model_joint_index_map_.find(model_lift_joint_name_);
  if (it != model_joint_index_map_.end()) {
    lift_joint_index_ = it->second;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                "Model lift joint '%s' not found in URDF.", model_lift_joint_name_.c_str());
  }
}

void LeaderController::rightTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  updateJointPositionsFromTrajectory(*msg);
  right_traj_received_ = true;
  last_right_traj_time_ = this->now();
}

void LeaderController::leftTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  updateJointPositionsFromTrajectory(*msg);
  left_traj_received_ = true;
  last_left_traj_time_ = this->now();
}

void LeaderController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  updateLiftJointFromJointState(*msg);
}

void LeaderController::updateJointPositionsFromTrajectory(
  const trajectory_msgs::msg::JointTrajectory & msg)
{
  if (msg.points.empty()) {
    return;
  }
  const auto & point = msg.points.front();
  if (point.positions.empty()) {
    return;
  }

  for (size_t i = 0; i < msg.joint_names.size(); ++i) {
    auto it = model_joint_index_map_.find(msg.joint_names[i]);
    if (it == model_joint_index_map_.end()) {
      continue;
    }
    const int model_index = it->second;
    if (model_index < 0 || model_index >= q_.size()) {
      continue;
    }
    if (i < point.positions.size()) {
      q_[model_index] = point.positions[i];
    }
    if (i < point.velocities.size()) {
      qdot_[model_index] = point.velocities[i];
    }
  }
}

void LeaderController::updateLiftJointFromJointState(const sensor_msgs::msg::JointState & msg)
{
  if (lift_joint_index_ < 0 || lift_joint_index_ >= q_.size()) {
    return;
  }

  for (size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] != lift_joint_name_) {
      continue;
    }
    if (i < msg.position.size()) {
      q_[lift_joint_index_] = msg.position[i];
      follower_lift_position_ = msg.position[i];
      lift_joint_received_ = true;
    }
    if (i < msg.velocity.size()) {
      qdot_[lift_joint_index_] = msg.velocity[i];
    }
    return;
  }
}

void LeaderController::reactivateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg || msg->data == reactivate_state_) {
    return;
  }

  reactivate_state_ = msg->data;
  RCLCPP_WARN(this->get_logger(),
    "Reactivate topic '%s' set to %s. %s",
    reactivate_topic_.c_str(),
    reactivate_state_ ? "true" : "false",
    reactivate_state_ ? "Enabling leader controller output." :
      "Disabling leader controller output.");
}

void LeaderController::controlLoopCallback()
{
  static int loop_count = 0;
  static int debug_count = 0;

  loop_count++;

  const rclcpp::Time now = this->now();
  const bool right_traj_has_publisher =
    (r_traj_sub_ && r_traj_sub_->get_publisher_count() > 0);
  const bool left_traj_has_publisher =
    (l_traj_sub_ && l_traj_sub_->get_publisher_count() > 0);

  if (!right_traj_has_publisher && !left_traj_has_publisher) {
    was_publishing_reference_ = false;
    if (debug_count++ % 100 == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "No publishers on trajectory topics; skipping goal pose publish");
    }
    return;
  }

  const bool right_recent =
    right_traj_has_publisher && right_traj_received_ &&
    (now - last_right_traj_time_).seconds() < command_timeout_;
  const bool left_recent =
    left_traj_has_publisher && left_traj_received_ &&
    (now - last_left_traj_time_).seconds() < command_timeout_;
  const bool has_recent_reference = right_recent || left_recent;

        // Wait until we have at least one arm trajectory message before publishing any goal pose.
  if (!has_recent_reference) {
    was_publishing_reference_ = false;
    if (debug_count++ % 100 == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Control loop waiting for joint trajectory commands...");
    }
    return;
  }

  if (!reactivate_state_) {
    was_publishing_reference_ = false;
    if (debug_count++ % 100 == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Control loop waiting for reactivate topic '%s' to become true...",
                    reactivate_topic_.c_str());
    }
    return;
  }

  was_publishing_reference_ = true;

  try {
    kinematics_solver_->updateState(q_, qdot_);

    const bool mapping_active =
      pivot_links_valid_ && (motion_scale_ != 1.0 || remap_to_follower_shoulder_);

    if (right_recent) {
      Eigen::Affine3d r_pose =
        computePoseInBaseFrame(kinematics_solver_->getPose(r_gripper_name_));
      Eigen::Affine3d r_elbow_pose =
        computePoseInBaseFrame(kinematics_solver_->getPose(r_elbow_name_));
      if (mapping_active) {
        const Eigen::Affine3d r_pivot =
          computePoseInBaseFrame(kinematics_solver_->getPose(r_pivot_name_));
        const Eigen::Vector3d r_out = targetPivot(r_target_pivot_xyz_, r_pivot);
        r_pose = mapToFollower(r_pose, r_pivot, r_out);
        r_elbow_pose = mapToFollower(r_elbow_pose, r_pivot, r_out);
      }
      r_goal_pose_pub_->publish(makePoseStamped(r_pose));
      r_elbow_pose_pub_->publish(makePoseStamped(r_elbow_pose));
    }

    if (left_recent) {
      Eigen::Affine3d l_pose =
        computePoseInBaseFrame(kinematics_solver_->getPose(l_gripper_name_));
      Eigen::Affine3d l_elbow_pose =
        computePoseInBaseFrame(kinematics_solver_->getPose(l_elbow_name_));
      if (mapping_active) {
        const Eigen::Affine3d l_pivot =
          computePoseInBaseFrame(kinematics_solver_->getPose(l_pivot_name_));
        const Eigen::Vector3d l_out = targetPivot(l_target_pivot_xyz_, l_pivot);
        l_pose = mapToFollower(l_pose, l_pivot, l_out);
        l_elbow_pose = mapToFollower(l_elbow_pose, l_pivot, l_out);
      }
      l_goal_pose_pub_->publish(makePoseStamped(l_pose));
      l_elbow_pose_pub_->publish(makePoseStamped(l_elbow_pose));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "FK computation failed: %s", e.what());
  }
}

geometry_msgs::msg::PoseStamped LeaderController::makePoseStamped(
  const Eigen::Affine3d & pose) const
{
  geometry_msgs::msg::PoseStamped msg;
        // msg.header.stamp = this->now();
  msg.header.frame_id = base_frame_id_;
  msg.pose.position.x = pose.translation().x();
  msg.pose.position.y = pose.translation().y();
  msg.pose.position.z = pose.translation().z();

  const Eigen::Quaterniond quat(pose.linear());
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();
  return msg;
}

Eigen::Vector3d LeaderController::targetPivot(
  const std::vector<double> & target_pivot_xyz, const Eigen::Affine3d & leader_pivot) const
{
  if (!remap_to_follower_shoulder_ || target_pivot_xyz.size() != 3) {
    return leader_pivot.translation();      // re-anchoring disabled: pivot stays on the leader
  }
  Eigen::Vector3d out(target_pivot_xyz[0], target_pivot_xyz[1], target_pivot_xyz[2]);
  out.z() += follower_lift_position_;       // follower shoulder rides on the lift
  return out;
}

Eigen::Affine3d LeaderController::mapToFollower(
  const Eigen::Affine3d & pose, const Eigen::Affine3d & leader_pivot,
  const Eigen::Vector3d & target_pivot) const
{
  // target = target_pivot + motion_scale * (pose - leader_pivot); orientation unchanged.
  // With target_pivot == leader_pivot and motion_scale == 1 this is the identity.
  Eigen::Affine3d mapped = pose;
  mapped.translation() =
    target_pivot + motion_scale_ * (pose.translation() - leader_pivot.translation());
  return mapped;
}

Eigen::Affine3d LeaderController::computePoseInBaseFrame(
  const Eigen::Affine3d & link_pose) const
{
  if (kinematics_solver_ && kinematics_solver_->hasLinkFrame("world")) {
    const Eigen::Affine3d base_pose = kinematics_solver_->getPose("world");
    return base_pose.inverse() * link_pose;
  }
  return link_pose;
}
}  // namespace cyclo_motion_controller_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cyclo_motion_controller_ros::LeaderController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
