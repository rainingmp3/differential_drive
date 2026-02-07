#include "pid.hpp"
#include <algorithm>

PIDController::PIDController(float kp, float kd, float ki, float max_windup,
                             float max_input, float time_step)
    : kp_(kp), ki_(ki), kd_(kd), prev_state_(0.0f), prev_error_(0.0f),
      max_windup_(max_windup), max_input_(max_input), integral_error_(0.0f),
      time_step_(time_step) {}

float PIDController::computeControl(float setpoint, float current_state) {
  float error = setpoint - current_state;
  float derivative_error = (current_state - prev_state_) / time_step_;
  integral_error_ += error * time_step_;
  integral_error_ = std::clamp(integral_error_, -max_windup_, max_windup_);

  float control_input =
      derivative_error * kd_ + integral_error_ * ki_ + error * kp_;
  control_input = std::clamp(control_input, -max_input_, max_input_);
  prev_error_ = error;
  prev_state_ = current_state;
  return control_input;
}
