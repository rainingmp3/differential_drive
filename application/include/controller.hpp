#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.hpp"
#include "wrap_angle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cstddef>
#include <math.h>
#include <algorithm>
#include <string>
#include <vector>
#include <limits>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:
  void publishTwist(float velocity, float angular_velocity);

  void analyzeScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  float QuatToYaw(float qx, float qy, float qz, float qw);

  void applyInputs();
  std::string log_msg;
  void publishLog(std::string &msg);
  // Set variables:
  bool obstacle_is_near = 0;
  float LIDAR_TO_FRONT =
      0.854283f; // Distance from Lidar to the front of the car.;
  float back_velocity = -1.0f;

  struct LidarPoint {
    float angle;
    float distance;
  };

  std::vector<LidarPoint> lidar_points;
  float position_x;
  float position_y;
  float position_z;

  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;
  float orientation_yaw;

  float goal_position_x;
  float goal_position_y;
  float goal_position_z;
  float goal_yaw;

  float goal_qx;
  float goal_qy;
  float goal_qz;
  float goal_qw;

  // Update rates [hz]
  float rate_lidar = 10;   // Lidar
  float rate_odom = 50;    // Wheel encoders
  float rate_control = 25; // Wheel encoders
  float rate_goal = 1;     // Wheel encoders
  // PID
  float kp_ = 0.2f;
  float ki_ = 0.0f;
  float kd_ = 0.01f;
  float max_windup_ = 1.0f;
  float max_input_ = 0.5f;
  PIDController pid_;

  rclcpp::TimerBase::SharedPtr timer_logs;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_scan;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_goal;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
};
