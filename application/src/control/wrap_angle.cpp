#include "wrap_angle.hpp"
#include <cmath>

const float PI = 3.14159265359;
float wrapAngle(float angle_to_wrap)
{
  float wrapped_angle = angle_to_wrap;
  if (angle_to_wrap > PI / 2)
    return wrapped_angle = angle_to_wrap - PI;
  else if (angle_to_wrap < -PI / 2)
    return wrapped_angle = angle_to_wrap + PI;
  else
    return wrapped_angle;
}
