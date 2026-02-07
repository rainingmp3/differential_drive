#pragma once

class PIDController {
public:
  PIDController(float kp, float kd, float ki, float max_windup, float max_input,
                float time_step);
  float computeControl(float setpoint, float current_state);
  float computeControl(float error);

private:
  float kp_, ki_, kd_;
  float time_step_;
  float max_windup_, max_input_;
  float integral_error_, prev_error_, prev_state_;
};
