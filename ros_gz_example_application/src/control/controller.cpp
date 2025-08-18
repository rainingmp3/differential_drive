// Publisher dependencies
#include <chrono>
#include <cstddef>

#include "geometry_msgs/msg/twist.hpp"
// rclcpp itself
#include "rclcpp/rclcpp.hpp"
// Subscriber dependencies
#include "sensor_msgs/msg/laser_scan.hpp"
using namespace std::chrono_literals;

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    auto laser_callback_ =
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
    { this->analyzeScan(msg); };

    auto timer_callback_ = [this]()
    { this->publishVelocity(desired_velocity); }; // Subsciber part

    vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive/cmd_vel", 10);

    timer_ = this->create_wall_timer(25ms, timer_callback_); // Subsciber part

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/diff_drive/scan", 1, laser_callback_);
  }

private:
  void publishVelocity(float velocity = 5.0f)
  {
    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = velocity;
    vel_publisher->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "Published velocity %f [m/s]", velocity);
  }
  void analyzeScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    size_t middle_index = msg->ranges.size() / 2;
    float distance_forward = msg->ranges[middle_index] - lidar_to_front;
    RCLCPP_INFO(this->get_logger(), "Distance is  %f [m]", distance_forward);
    if (distance_forward < lidar_to_front + 0.5)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "WE SURPRASSED IT");
      this->desired_velocity = 0.0f;
    }
  }
  // Set variables:
  float desired_velocity = 5.0f;
  float lidar_to_front = 0.854283f;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
