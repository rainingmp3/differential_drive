#include "potential_field.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

PotentialField::PotentialField(float g_star, float ka, float kr, float kpf)
    : g_star_(g_star), ka_(ka), kr_(kr), kpf_(kpf)
{
}

Eigen::Vector2f PotentialField::attractionForce(float x_body, float y_body,
                                                float x_desired,
                                                float y_desired)
{
  return -ka_ / 2 * Eigen::Vector2f(x_body - x_desired, y_body - y_desired);
};

Eigen::Vector2f PotentialField::repulsiveForce(float x_body, float y_body,
                                               float x_object, float y_object)
{
  float g = sqrt(pow(x_body - x_object, 2) + pow(y_body - y_object, 2));
  if (g <= g_star_ && g > 1e-6)
  {
    return kr_ * (1 / g - 1 / g_star_) * pow(g, -3) *
           Eigen::Vector2f(x_body - x_object, y_body - y_object);
  }
  else
    return Eigen::Vector2f(0, 0);
};

Eigen::Vector2f PotentialField::computeTwist(Eigen::Vector2f attractive_force, 
                               Eigen::Vector2f repulsive_force)
{
  Eigen::Vector2f result_force = attractive_force + repulsive_force;
  float linear_velocity = kpf_ * result_force.norm();
  float yaw = std::atan2(result_force(1),result_force(0));
  return Eigen::Vector2f(linear_velocity,yaw);
}
