#include "controller.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>

ControllerNode::ControllerNode()
    : Node("controller_node"), pid_(kp_, ki_, kd_, max_windup_, max_input_),
      lqr_()

{ // NOTE: SOME NONSTATIC CPP BULLSHIT
  A = Eigen::Matrix3f::Identity();
  Q = Eigen::Matrix3f::Identity();
  Q(2, 2) = 2.f;
  R = Eigen::Matrix2f::Identity() * 0.1f;

  // Callback function, are being called on trigger
  // Laser callback func
  auto laser_callback_ =
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Make sure no previous lidar points from previous scan are included
    this->lidar_points.clear();
    this->analyzeScan(msg);
  };

  // Goal callback, being called everytime goal point is set in Rviz
  auto goal_callback_ =
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Extracting goal position and heading information
    this->goal_position_x = msg->pose.position.x;
    this->goal_position_y = msg->pose.position.y;
    this->goal_position_z = msg->pose.position.z;

    // Wrapping angle around pi/2 allows to drive backwards
    this->goal_yaw = wrapAngle(
        atan2(goal_position_y - position_y, goal_position_x - position_x));
  };

  // Applies inputs on set timestep
  auto timer_callback_ = [this]() { this->applyInputs(); }; // Subsciber part

  // Extracting odometry informaton on position and orientation via dead
  // reckoning
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

  // Publsihers declarations, return publisher pointer
  twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive/cmd_vel", rate_control);

  timer_ = this->create_wall_timer(40ms, timer_callback_);
  debug_publisher = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/debug", rate_control);

  subscription_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/diff_drive/scan", rate_lidar, laser_callback_);

  subscription_goal =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", rate_goal, goal_callback_);

  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive/odometry", rate_odom, odom_callback_);
}
void ControllerNode::publishDebug(double x, double y, double z)
// Publishes linear and angular velocity
{
  auto debug_msg = geometry_msgs::msg::Vector3();
  debug_msg.x = x;
  debug_msg.y = y;
  debug_msg.z = z;

  debug_publisher->publish(debug_msg);
}

void ControllerNode::publishTwist(float velocity, float angular_velocity)
// Publishes linear and angular velocity
{
  auto vel_msg = geometry_msgs::msg::Twist();
  vel_msg.linear.x = velocity;
  vel_msg.angular.z = angular_velocity;
  twist_publisher->publish(vel_msg);
}

void ControllerNode::analyzeScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
// Analyzes distance and angle to every scan point,
// returns struct consisting of these two
{
  size_t max_index = msg->ranges.size();
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

  // RCLCPP_WARN(this->get_logger(), "Right angle is %f ; right angle is %f",
  //             lidar_points[0].angle * 180 / M_PI,
  //             lidar_points[lidar_points.size() - 1].angle * 180 / M_PI);
}
float ControllerNode::QuatToYaw(float qx, float qy, float qz, float qw)
{
  float yaw = atan2(2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz)); // TODO: learn it pls
  yaw = (yaw);
  return yaw;
}

void ControllerNode::applyInputs()
// Parses velocity commands to publishTwist functions
{
  Vector3f actual_state(position_x, position_y, wrapAngle(orientation_yaw));
  Vector3f desired_state(goal_position_x, goal_position_y, wrapAngle(goal_yaw));
  Matrix<float, 3, 2> B = lqr_.computeB(actual_state, 1 / rate_control);
  RCLCPP_WARN(this->get_logger(), "desired_state : %f %f %f", goal_position_x,
              goal_position_y, goal_yaw);

  RCLCPP_WARN(this->get_logger(), "actual_state : %f %f %f", position_x,
              position_y, orientation_yaw);
  Vector2f optimal_input = lqr_.computeInput(A, B, Q, R, desired_state,
                                             actual_state, 1 / rate_control);
  double err_x = actual_state(0) - desired_state(0);
  double err_y = actual_state(1) - desired_state(1);
  double err_z = actual_state(2) - desired_state(2);
  publishDebug(err_x, err_y, err_z);
  this->publishTwist(optimal_input(0), optimal_input(1));
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
