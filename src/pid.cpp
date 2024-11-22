#include <limits>
#include "pid.h"

PID::PID(float kp, float ki, float kd, float dt) :
  kp(kp), ki(ki), kd(kd), dt(dt), integral(0), prev_error(0),
  low(std::numeric_limits<float>::min()), high(std::numeric_limits<float>::max()) {
}

float PID::Update(float error) {
  return Update(error, this->dt);
}

float PID::Update(float error, float dt) {
  float proportional = kp * error;
  integral += error * ki * dt;
  float derivative = (error - prev_error) * kd / dt;
  prev_error = error;

  float output = proportional + integral + derivative;

  if (output > high) {
    output = high;
    integral = high - proportional - derivative; // limit integral to prevent windup
  }
  else if (output < low) {
    output = low;
    integral = low - proportional - derivative; // limit integral to prevent windup
  }
  return output;
}

void PID::SetGains(float kp, float ki, float kd)
{
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PID::SetOutputLimits(float low, float high)
{
  this->low = low;
  this->high = high;
}