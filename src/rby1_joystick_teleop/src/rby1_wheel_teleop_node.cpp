#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RBY1WheelTeleopNode : public rclcpp::Node
{
public:
  RBY1WheelTeleopNode()
  : Node("rby1_wheel_teleop_node")
  {
    joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/isaac_wheel_commands");

    left_wheel_name_ = this->declare_parameter<std::string>("left_wheel_name", "left_wheel");
    right_wheel_name_ = this->declare_parameter<std::string>("right_wheel_name", "right_wheel");

    // Logitech F710 기준:
    // left stick horizontal = axes[0]
    // left stick vertical   = axes[1]
    linear_axis_ = this->declare_parameter<int>("linear_axis", 1);
    angular_axis_ = this->declare_parameter<int>("angular_axis", 0);

    max_wheel_speed_rad_s_ = this->declare_parameter<double>("max_wheel_speed_rad_s", 2.0);
    deadzone_ = this->declare_parameter<double>("deadzone", 0.08);
    control_hz_ = this->declare_parameter<double>("control_hz", 60.0);

    invert_linear_ = this->declare_parameter<bool>("invert_linear", false);
    invert_angular_ = this->declare_parameter<bool>("invert_angular", false);
    invert_left_wheel_ = this->declare_parameter<bool>("invert_left_wheel", false);
    invert_right_wheel_ = this->declare_parameter<bool>("invert_right_wheel", false);

    linear_cmd_ = 0.0;
    angular_cmd_ = 0.0;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      10,
      std::bind(&RBY1WheelTeleopNode::JoyCallback, this, std::placeholders::_1)
    );

    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      command_topic_,
      10
    );

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&RBY1WheelTeleopNode::ControlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "RBY1 wheel teleop node started.");
    RCLCPP_INFO(this->get_logger(), "Command mode: WHEEL VELOCITY COMMAND ONLY");
    RCLCPP_INFO(this->get_logger(), "joy_topic     : %s", joy_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "command_topic : %s", command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "left stick vertical   -> forward/backward");
    RCLCPP_INFO(this->get_logger(), "left stick horizontal -> turn left/right");
  }

private:
  double ApplyDeadzone(const double value) const
  {
    if (std::fabs(value) < deadzone_) {
      return 0.0;
    }
    return value;
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double linear = 0.0;
    double angular = 0.0;

    if (linear_axis_ >= 0 && linear_axis_ < static_cast<int>(msg->axes.size())) {
      linear = ApplyDeadzone(static_cast<double>(msg->axes[linear_axis_]));
    }

    if (angular_axis_ >= 0 && angular_axis_ < static_cast<int>(msg->axes.size())) {
      angular = ApplyDeadzone(static_cast<double>(msg->axes[angular_axis_]));
    }

    if (invert_linear_) {
      linear *= -1.0;
    }

    if (invert_angular_) {
      angular *= -1.0;
    }

    linear_cmd_ = linear;
    angular_cmd_ = angular;
  }

  void ControlLoop()
  {
    const rclcpp::Time now = this->now();

    // Differential drive wheel velocity command
    //
    // forward/backward:
    //   left  = linear
    //   right = linear
    //
    // turn:
    //   left  = -angular
    //   right = +angular
    //
    double left_wheel_vel =
      max_wheel_speed_rad_s_ * (linear_cmd_ - angular_cmd_);

    double right_wheel_vel =
      max_wheel_speed_rad_s_ * (linear_cmd_ + angular_cmd_);

    if (invert_left_wheel_) {
      left_wheel_vel *= -1.0;
    }

    if (invert_right_wheel_) {
      right_wheel_vel *= -1.0;
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now;

    cmd.name = {
      left_wheel_name_,
      right_wheel_name_
    };

    // Wheel은 velocity command만 사용
    cmd.position.clear();

    cmd.velocity = {
      left_wheel_vel,
      right_wheel_vel
    };

    cmd.effort.clear();

    cmd_pub_->publish(cmd);
  }

private:
  std::string joy_topic_;
  std::string command_topic_;

  std::string left_wheel_name_;
  std::string right_wheel_name_;

  int linear_axis_;
  int angular_axis_;

  double max_wheel_speed_rad_s_;
  double deadzone_;
  double control_hz_;

  bool invert_linear_;
  bool invert_angular_;
  bool invert_left_wheel_;
  bool invert_right_wheel_;

  double linear_cmd_;
  double angular_cmd_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RBY1WheelTeleopNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}