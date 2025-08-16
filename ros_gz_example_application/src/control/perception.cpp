#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserSubscriber : public rclcpp::Node {
 public:
  LaserSubscriber() : Node("laser_subscriber") {
    auto callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // RCLCPP_INFO(this->get_logger(), "Got a sensor msg %f", msg->scan_time);
      RCLCPP_INFO(this->get_logger(), "Got a sensor msg");
    };
    subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>("/diff_drive/scan", 1, callback);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserSubscriber>());
  rclcpp::shutdown();
  return 0;
}
