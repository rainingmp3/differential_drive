#pragma once
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

class PotentialField
{
public:
  PotentialField(float g_star, float ka, float kr, float kpf);
  Eigen::Vector2f attractionForce(float x_body, float y_body, float x_desired,
                                  float y_desired);
  Eigen::Vector2f repulsiveForce(float x_body, float y_body, float x_object,
                                 float y_object);
  Eigen::Vector2f computeTwist(Eigen::Vector2f attractive_force, 
                               Eigen::Vector2f repulsive_force);

private:
  float g_star_, ka_, kr_, kpf_;
};
