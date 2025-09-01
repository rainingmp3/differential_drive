#include "controller.hpp"
#include "wrap_angle.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>

ControllerNode::ControllerNode()
    : Node("controller_node"), pid_(kp_, ki_, kd_, max_windup_, max_input_),
      pf_(g_star_, ka_, kr_, kpf_)

{
  auto laser_callback_ =
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    this->lidar_points.clear();
    this->sum_repulsive_force = Eigen::Vector2f(0, 0);
    this->analyzeScan(msg);
  };

  auto goal_callback_ =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    this->goal_is_set = true;
    this->goal_position_x = msg->pose.position.x;
    this->goal_position_y = msg->pose.position.y;

    this->goal_yaw = wrapAngle(
        atan2(goal_position_y - position_y, goal_position_x - position_x));
  };

  auto timer_logs_callback_ = [this]()
  { this->publishLog(log_msg); }; // Subsciber part

  auto timer_callback_ = [this]() { this->applyInputs(); }; // Subsciber part

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

  timer_ = this->create_wall_timer(40ms, timer_callback_);
  timer_logs = this->create_wall_timer(1000ms, timer_logs_callback_);

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
  max_index = msg->ranges.size();
  float min_angle = msg->angle_min;
  float angle_increment = msg->angle_increment;

  for (int i = 0; i < max_index; ++i)
  {
    if (std::isnormal(msg->ranges[i]))
    {
      LidarPoint point;
      point.distance = msg->ranges[i];
      point.angle = min_angle + angle_increment * i;
      lidar_points.push_back(point);
    }
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
  {
    sum_repulsive_force = Eigen::Vector2f(0, 0);

    for (int i = 0; i < max_index; ++i)
    {
      float x_object =
          position_x + lidar_points[i].distance *
                           cos(orientation_yaw + lidar_points[i].angle);
      float y_object =
          position_y + lidar_points[i].distance *
                           sin(orientation_yaw + lidar_points[i].angle);
      sum_repulsive_force +=
          pf_.repulsiveForce(position_x, position_y, x_object, y_object);
    }

    Eigen::Vector2f attractive_force(0, 0);
    if (goal_is_set)
    {
      attractive_force = pf_.attractionForce(position_x, position_y,
                                             goal_position_x, goal_position_y);
    }
    else
    {
      attractive_force = Eigen::Vector2f(0, 0);
    }

    Eigen::Vector2f twist =
        pf_.computeTwist(attractive_force, Eigen::Vector2f(0, 0));

    float control_velocity = 0.1 * twist(0);

    float control_yaw_velocity = 0.5 * wrapAngle(twist(1));

    RCLCPP_WARN(this->get_logger(), "af = %f goal = %f posx = %f",
                attractive_force(0), goal_position_x, position_x);
    this->publishTwist(control_velocity, control_yaw_velocity);
  }
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
