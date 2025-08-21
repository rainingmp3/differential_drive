#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cstddef>

using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
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
    };

    auto timer_callback_ = [this]()
    { this->publishVelocity(desired_velocity); }; // Subsciber part

    auto odom_callback_ = [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      this->position_x = msg->pose.pose.position.x;
      this->position_y = msg->pose.pose.position.y;
      this->position_z = msg->pose.pose.position.z;

      this->orientation_x = msg->pose.pose.orientation.x;
      this->orientation_y = msg->pose.pose.orientation.y;
      this->orientation_z = msg->pose.pose.orientation.z;
      this->orientation_w = msg->pose.pose.orientation.w;
    };

    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive/cmd_vel", 10);

    timer_ = this->create_wall_timer(25ms, timer_callback_); // Subsciber part

    subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/diff_drive/scan", 1, laser_callback_);

    subscription_goal =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1, goal_callback_);

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive/odometry", 1, goal_callback_);
  }

private:
  void publishVelocity(float velocity)
  {
    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = velocity;
    vel_publisher->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "Published velocity %f [m/s]", velocity);
  }

  void analyzeScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    size_t middle_index = msg->ranges.size() / 2;
    float distance_forward = msg->ranges[middle_index] - LIDAR_TO_FRONT;
    RCLCPP_INFO(this->get_logger(), "Distance is  %f [m]", distance_forward);
    if (distance_forward < LIDAR_TO_FRONT + 0.5)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "WE SURPRASSED IT");
      this->desired_velocity = 0.0f;
    }

    // void PIDController(
    //   k_p,k_i,k_d,
    // ){
    //   //
    }
  }
  // Set variables:
  float LIDAR_TO_FRONT = 0.854283f; // Distance from Lidar to the front of the car.
  float desired_velocity = 5.0f;

  float position_x;
  float position_y;
  float position_z;

  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;

  float goal_position_x;
  float goal_position_y;
  float goal_position_z;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
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
