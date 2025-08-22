#include <cmath>
#inclide "wrapped_angle.hpp"
const float PI = 3.14159265359;
float wrapAngle(float angle_to_wrap)
{
  // return (angles + np.pi)%(2 * np.pi) - np.pi
  float wrapped_angle = fmod(angle_to_wrap + PI, 2 * PI) - PI;
  return wrapped_angle;
}
