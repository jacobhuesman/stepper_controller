#ifndef STEPPER_H
#define STEPPER_H

#include <gpio.h>
#include <stdexcept>

class Stepper
{
public:
  Stepper(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);
  void init();
  void enable();
  void disable();
  void cw();
  void ccw();
  void step();
  void zero();
  bool isZeroed();
  uint16_t getTicks();
  void setVelocity(float velocity);
  float velocitySetpoint();
  void minLimit();
  void maxLimit();
private:
  TIM_HandleTypeDef *htim1, *htim2;
  GPIO l0, l1, dm0, dm1, dm2, rst, dir, en;
  float velocity_setpoint, max_velocity; // rev/s
  bool zeroed, scanning;
};

#endif // STEPPER_H
