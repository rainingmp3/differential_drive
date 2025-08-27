#include "controller.hpp"
#include "wrap_angle.hpp"
#include <cmath>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>

ControllerNode::ControllerNode()
    : Node("controller_node"), pid_(kp_, ki_, kd_, max_windup_, max_input_)

{
  auto laser_callback_ =
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
  { this->analyzeScan(msg); };

  auto goal_callback_ =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    this->goal_position_x = msg->pose.position.x;
    this->goal_position_y = msg->pose.position.y;
    this->goal_position_z = msg->pose.position.z;

    this->goal_yaw = wrapAngle(
        atan2(goal_position_y - position_y, goal_position_x - position_x));
  };

  auto timer_logs_callback_ = [this]()
  {
    std::string log_msg =
        "yaw = " + std::to_string(this->orientation_yaw * 180 / M_PI) +
        " goal yaw = " + std::to_string(this->goal_yaw * 180 / M_PI) +
        " position_x = " + std::to_string(this->position_x) +
        " goal position_x = " + std::to_string(this->goal_position_x);

    this->publishLog(log_msg);
  }; // Subsciber part

  auto timer_callback_ = [this]() { this->applyInputs(); }; // Subsciber part

  // TEST TEST TEST
  // TEST TEST TEST
  auto odom_callback_ = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    this->position_x = msg->pose.pose.position.x;
    this->position_y = msg->pose.pose.position.y;
    this->position_z = msg->pose.pose.position.z;

    this->orientation_x = msg->pose.pose.orientation.x;
    this->orientation_y = msg->pose.pose.orientation.y;
    this->orientation_z = msg->pose.pose.orientation.z;
    this->orientation_w = msg->pose.pose.orientation.w;

    this->orientation_yaw =
        QuatToYaw(orientation_x, orientation_y, orientation_z, orientation_w);
  };

  twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive/cmd_vel", rate_control);

  timer_ = this->create_wall_timer(40ms, timer_callback_); // TODO: avoid 25ms
  timer_logs =
      this->create_wall_timer(1000ms, timer_logs_callback_); // TODO: avoid 25ms

  subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/diff_drive/scan", rate_lidar, laser_callback_);

  subscription_goal =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", rate_goal, goal_callback_);

  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive/odometry", rate_odom, odom_callback_);
}

void ControllerNode::publishTwist(float velocity, float angular_velocity)
{
  auto vel_msg = geometry_msgs::msg::Twist();
  vel_msg.linear.x = velocity;
  vel_msg.angular.z = angular_velocity;
  twist_publisher->publish(vel_msg);
}

void ControllerNode::analyzeScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  size_t middle_index = msg->ranges.size() / 2;
  float distance_forward = msg->ranges[middle_index] - LIDAR_TO_FRONT;
  // RCLCPP_WARN(this->get_logger(), "Distance is  %f [m]", distance_forward);
  if (distance_forward < LIDAR_TO_FRONT + 0.5)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "WE SURPRASSED IT");
    this->desired_velocity = 0.0f;
  }
}
float ControllerNode::QuatToYaw(float qx, float qy, float qz, float qw)
{
  float yaw = atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz)); // TODO: learn it pls
  yaw = (yaw);
  return yaw;
}

void ControllerNode::applyInputs()
{
  float control_velocity =
      pid_.computeControl(goal_position_x, position_x, rate_control);

  float control_yaw_velocity =
      pid_.computeControl(goal_yaw, orientation_yaw, rate_control);

  this->publishTwist(control_velocity, control_yaw_velocity);
  //   // this->publishTwist(control_velocity, 0);
  // if (obstacle_is_near)
  // {
  //   this->publishTwist(desired_velocity, control_yaw_velocity);
  //   // this->publishTwist(desired_velocity, 0);
  // }
  // else
  // {
  //
  //   this->publishTwist(control_velocity, control_yaw_velocity);
  //   // this->publishTwist(control_velocity, 0);
  // }
}

void ControllerNode::publishLog(std::string &msg)
{
  RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
