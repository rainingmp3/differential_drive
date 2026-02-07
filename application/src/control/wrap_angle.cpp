#include "wrap_angle.hpp"
#include <cmath>

float wrapAngle(float angle_to_wrap) {
  float wrapped_angle = angle_to_wrap;
  if (angle_to_wrap > PI / 2)
    return wrapped_angle = angle_to_wrap - PI;
  else if (angle_to_wrap < -PI / 2)
    return wrapped_angle = angle_to_wrap + PI;
  else
    return wrapped_angle;
}
