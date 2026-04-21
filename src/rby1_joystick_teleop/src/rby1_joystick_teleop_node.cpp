#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RBY1JoystickTeleopNode : public rclcpp::Node
{
public:
  RBY1JoystickTeleopNode()
  : Node("rby1_joystick_teleop_node")
  {
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/isaac_joint_states");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/isaac_joint_commands");

    control_hz_ = this->declare_parameter<double>("control_hz", 60.0);
    position_rate_rad_s_ = this->declare_parameter<double>("position_rate_rad_s", 0.5);
    joint_limit_rad_ = this->declare_parameter<double>("joint_limit_rad", 3.141592653589793);

    dpad_x_axis_ = this->declare_parameter<int>("dpad_x_axis", 6);
    dpad_y_axis_ = this->declare_parameter<int>("dpad_y_axis", 7);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.5);

    // Logitech F710 기준으로 보통 LB=4, RB=5
    left_arm_button_ = this->declare_parameter<int>("left_arm_button", 4);
    right_arm_button_ = this->declare_parameter<int>("right_arm_button", 5);

    invert_increment_ = this->declare_parameter<bool>("invert_increment", false);

    left_arm_joint_names_ = {
      "left_arm_0",
      "left_arm_1",
      "left_arm_2",
      "left_arm_3",
      "left_arm_4",
      "left_arm_5",
      "left_arm_6"
    };

    right_arm_joint_names_ = {
      "right_arm_0",
      "right_arm_1",
      "right_arm_2",
      "right_arm_3",
      "right_arm_4",
      "right_arm_5",
      "right_arm_6"
    };

    active_side_ = "left";
    active_joint_index_ = 0;

    target_initialized_ = false;

    increment_cmd_ = 0.0;
    select_cmd_ = 0.0;
    prev_select_cmd_ = 0.0;

    last_time_ = this->now();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&RBY1JoystickTeleopNode::JoyCallback, this, std::placeholders::_1)
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      10,
      std::bind(&RBY1JoystickTeleopNode::JointStateCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      command_topic_,
      10
    );

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&RBY1JoystickTeleopNode::ControlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "RBY1 joystick teleop node started.");
    RCLCPP_INFO(this->get_logger(), "Mode: ARM JOINT POSITION COMMAND");
    RCLCPP_INFO(this->get_logger(), "D-pad up/down    : increment/decrement selected joint");
    RCLCPP_INFO(this->get_logger(), "D-pad left/right : change selected joint index");
    RCLCPP_INFO(this->get_logger(), "LB/RB            : select left/right arm");
    RCLCPP_INFO(this->get_logger(), "command_topic    : %s", command_topic_.c_str());
  }

private:
  double Clamp(const double value, const double min_value, const double max_value) const
  {
    return std::max(min_value, std::min(value, max_value));
  }

  double AxisToDiscrete(const double value) const
  {
    if (value > deadzone_) {
      return 1.0;
    }
    if (value < -deadzone_) {
      return -1.0;
    }
    return 0.0;
  }

  bool IsButtonPressed(const sensor_msgs::msg::Joy::SharedPtr msg, const int index) const
  {
    if (index < 0) {
      return false;
    }
    if (index >= static_cast<int>(msg->buttons.size())) {
      return false;
    }
    return msg->buttons[index] != 0;
  }

  std::vector<std::string> CurrentArmJointNames() const
  {
    if (active_side_ == "right") {
      return right_arm_joint_names_;
    }
    return left_arm_joint_names_;
  }

  std::vector<double> & CurrentArmTargets()
  {
    if (active_side_ == "right") {
      return right_arm_targets_;
    }
    return left_arm_targets_;
  }

  const std::vector<double> & CurrentArmTargetsConst() const
  {
    if (active_side_ == "right") {
      return right_arm_targets_;
    }
    return left_arm_targets_;
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double dpad_x = 0.0;
    double dpad_y = 0.0;

    if (dpad_x_axis_ >= 0 && dpad_x_axis_ < static_cast<int>(msg->axes.size())) {
      dpad_x = AxisToDiscrete(static_cast<double>(msg->axes[dpad_x_axis_]));
    }

    if (dpad_y_axis_ >= 0 && dpad_y_axis_ < static_cast<int>(msg->axes.size())) {
      dpad_y = AxisToDiscrete(static_cast<double>(msg->axes[dpad_y_axis_]));
    }

    if (invert_increment_) {
      dpad_y *= -1.0;
    }

    increment_cmd_ = dpad_y;
    select_cmd_ = dpad_x;

    if (IsButtonPressed(msg, left_arm_button_)) {
      active_side_ = "left";
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Selected arm: LEFT"
      );
    }

    if (IsButtonPressed(msg, right_arm_button_)) {
      active_side_ = "right";
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Selected arm: RIGHT"
      );
    }
  }

  void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      current_joint_pos_[msg->name[i]] = msg->position[i];
    }

    if (target_initialized_) {
      return;
    }

    bool left_ready = true;
    bool right_ready = true;

    for (const auto & name : left_arm_joint_names_) {
      if (current_joint_pos_.find(name) == current_joint_pos_.end()) {
        left_ready = false;
        break;
      }
    }

    for (const auto & name : right_arm_joint_names_) {
      if (current_joint_pos_.find(name) == current_joint_pos_.end()) {
        right_ready = false;
        break;
      }
    }

    if (!left_ready || !right_ready) {
      return;
    }

    left_arm_targets_.clear();
    right_arm_targets_.clear();

    for (const auto & name : left_arm_joint_names_) {
      left_arm_targets_.push_back(
        Clamp(current_joint_pos_[name], -joint_limit_rad_, joint_limit_rad_)
      );
    }

    for (const auto & name : right_arm_joint_names_) {
      right_arm_targets_.push_back(
        Clamp(current_joint_pos_[name], -joint_limit_rad_, joint_limit_rad_)
      );
    }

    target_initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "Initial arm joint targets loaded from /isaac_joint_states.");
    PrintStatus();
  }

  void ControlLoop()
  {
    if (!target_initialized_) {
      return;
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 0.2) {
      dt = 1.0 / control_hz_;
    }

    // D-pad left/right edge detection for joint selection
    if (select_cmd_ != 0.0 && prev_select_cmd_ == 0.0) {
      if (select_cmd_ > 0.0) {
        active_joint_index_ -= 1;
      } else {
        active_joint_index_ += 1;
      }

      if (active_joint_index_ < 0) {
        active_joint_index_ = 6;
      }

      if (active_joint_index_ > 6) {
        active_joint_index_ = 0;
      }

      PrintStatus();
    }
    prev_select_cmd_ = select_cmd_;

    auto & targets = CurrentArmTargets();

    if (active_joint_index_ >= 0 && active_joint_index_ < static_cast<int>(targets.size())) {
      targets[active_joint_index_] += position_rate_rad_s_ * increment_cmd_ * dt;
      targets[active_joint_index_] = Clamp(
        targets[active_joint_index_],
        -joint_limit_rad_,
        joint_limit_rad_
      );
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now;

    cmd.name = CurrentArmJointNames();
    cmd.position = CurrentArmTargetsConst();

    cmd.velocity.clear();
    cmd.effort.clear();

    cmd_pub_->publish(cmd);
  }

  void PrintStatus()
  {
    const auto names = CurrentArmJointNames();
    const auto & targets = CurrentArmTargetsConst();

    if (active_joint_index_ < 0 || active_joint_index_ >= static_cast<int>(names.size())) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[%s arm] selected joint: %s, target=%.6f rad",
      active_side_.c_str(),
      names[active_joint_index_].c_str(),
      targets[active_joint_index_]
    );
  }

private:
  std::string joy_topic_;
  std::string joint_state_topic_;
  std::string command_topic_;

  double control_hz_;
  double position_rate_rad_s_;
  double joint_limit_rad_;
  double deadzone_;

  int dpad_x_axis_;
  int dpad_y_axis_;
  int left_arm_button_;
  int right_arm_button_;

  bool invert_increment_;
  bool target_initialized_;

  std::string active_side_;
  int active_joint_index_;

  double increment_cmd_;
  double select_cmd_;
  double prev_select_cmd_;

  rclcpp::Time last_time_;

  std::vector<std::string> left_arm_joint_names_;
  std::vector<std::string> right_arm_joint_names_;

  std::vector<double> left_arm_targets_;
  std::vector<double> right_arm_targets_;

  std::unordered_map<std::string, double> current_joint_pos_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RBY1JoystickTeleopNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}