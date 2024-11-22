#pragma once

class PID
{
  public:
    PID(float kp, float ki, float kd, float dt);
    float Update(float error);
    float Update(float error, float dt);

    void SetOutputLimits(float low, float high);
    void SetGains(float kp, float ki, float kd);

  private:	
	float kp;
    float ki;
    float kd;
    float dt;
    float low;
    float high;

    float integral;
    float prev_error;
};
