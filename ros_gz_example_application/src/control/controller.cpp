#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cstddef>
#include <math.h>
#include <rclcpp/logging.hpp>
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
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

      this->goal_yaw =
          atan2(goal_position_y - position_y, goal_position_x - position_x);
      RCLCPP_ERROR(this->get_logger(), "goal yaw is %f", goal_yaw);
    };

    auto timer_callback_ = [this]()
    // TODO: Change to computeInputs
    { this->applyInputs(); }; // Subsciber part

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

      RCLCPP_WARN(this->get_logger(), "current yaw is %f", orientation_yaw);
    };

    twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive/cmd_vel", rate_control);

    timer_ = this->create_wall_timer(40ms, timer_callback_); // TODO: avoid 25ms

    subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/diff_drive/scan", rate_lidar, laser_callback_);

    subscription_goal =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rate_goal, goal_callback_);

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive/odometry", rate_odom, odom_callback_);
  }

private:
  void publishTwist(float velocity, float angular_velocity)
  {
    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = velocity;
    vel_msg.angular.z = angular_velocity;
    twist_publisher->publish(vel_msg);
    RCLCPP_WARN(this->get_logger(), "Published velocity %f [m/s]", velocity);
  }

  void analyzeScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    size_t middle_index = msg->ranges.size() / 2;
    float distance_forward = msg->ranges[middle_index] - LIDAR_TO_FRONT;
    RCLCPP_WARN(this->get_logger(), "Distance is  %f [m]", distance_forward);
    if (distance_forward < LIDAR_TO_FRONT + 0.5)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "WE SURPRASSED IT");
      this->desired_velocity = 0.0f;
    }
  }
  float QuatToYaw(float qx, float qy, float qz, float qw)
  {
    float yaw = atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz)); // TODO: learn it pls
    return yaw;
  }

  void applyInputs()
  {
    float control_velocity =
        pid_.computeControl(goal_position_x, position_x, rate_control);

    float control_yaw_velocity =
        pid_.computeControl(goal_yaw, orientation_yaw, rate_control);

    RCLCPP_WARN(this->get_logger(), "control angular_velocity is %f",
                control_yaw_velocity);
    if (obstacle_is_near)
    {
      this->publishTwist(desired_velocity, control_yaw_velocity);
    }
    else
    {

      this->publishTwist(control_velocity, control_yaw_velocity);
    }
  }

  // Set variables:
  bool obstacle_is_near = 0;
  float LIDAR_TO_FRONT =
      0.854283f; // Distance from Lidar to the front of the car.
  float desired_velocity = 5.0f;
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
  // TODO:
  // float control_input_x_ = pid_.computeControl(goal_position_x, position_x,
  // time_step)
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscription_scan;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_goal;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
