#include "wrap_angle.hpp"
#include <cmath>

const float PI = 3.14159265359;
float wrapAngle(float angle_to_wrap)
{
  if (angle_to_wrap > PI / 2)
  {
    int n = (int)(angle_to_wrap / PI + 1);
    angle_to_wrap -= n * PI;
  }
  else if (angle_to_wrap < -PI / 2)
  {
    int n = (int)(angle_to_wrap / PI - 1);
    angle_to_wrap += n * PI;
  }
  return angle_to_wrap;
}
