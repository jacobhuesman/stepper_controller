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
private:
  TIM_HandleTypeDef *htim1, *htim2;
  GPIO l0, l1, dm0, dm1, dm2, rst, dir, en;
  float max_velocity; // rev/s
  bool zeroed;
};

#endif // STEPPER_H
