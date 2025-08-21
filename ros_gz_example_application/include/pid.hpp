#ifndef PID_HPP
#define PID_HPP
class PIDController
{
public:
  PIDController(float kp, float kd, float ki, float max_windup, float max_input);
  float computeControl(float setpoint, float current_state, float time_step);

private:
  float kp_, ki_, kd_;
  float max_windup_, max_input_;
  float integral_error_, prev_error_;
};
#endif // !PID.HPP

