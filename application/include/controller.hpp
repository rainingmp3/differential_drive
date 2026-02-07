#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "lqr.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "wrap_angle.hpp"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <limits>
#include <math.h>
#include <string>
#include <vector>

using namespace std::chrono_literals;
class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();

private:
  void publishTwist(float velocity, float angular_velocity);
  void publishDebug(double x, double y, double z);

  void analyzeScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  float QuatToYaw(float qx, float qy, float qz, float qw);
  void applyInputs();

  // Set variables:
  bool obstacle_is_near = 0;
  float LIDAR_TO_FRONT =
      0.854283f; // Distance from Lidar to the front of the car.;

  struct LidarPoint {
    float angle;
    float distance;
  };

  std::vector<LidarPoint> lidar_points;
  float position_x = 0;
  float position_y = 0;
  float position_z = 0;

  float orientation_x = 0;
  float orientation_y = 0;
  float orientation_z = 0;
  float orientation_w = 0;
  float orientation_yaw = 0;

  float goal_position_x = 0;
  float goal_position_y = 0;
  float goal_position_z = 0;
  float goal_yaw = 0;

  float goal_qx = 0;
  float goal_qy = 0;
  float goal_qz = 0;
  float goal_qw = 0;

  // Update rates [hz]
  float rate_lidar = 10;   // Lidar
  float rate_odom = 50;    // Wheel encoders
  float rate_control = 25; // Wheel encoders
  float rate_goal = 1;     // Wheel encoders
  // PID linear
  float kp_lin = 0.8f;
  float ki_lin = 0.1f;
  float kd_lin = 0.2f;
  float max_windup_lin = 1.0f;
  float max_input_lin = 0.5f;
  // PID angular
  float kp_ang = 0.8f;
  float ki_ang = 0.1f;
  float kd_ang = 0.2f;
  float max_windup_ang = 1.0f;
  float max_input_ang = 0.5f;
  float time_step = 0.04;
  PIDController pid_linear_;
  PIDController pid_angular_;
  LQR lqr_;
  Eigen::Matrix3f A;
  Matrix3f Q;
  Matrix2f R;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr debug_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_scan;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_goal;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
};
