#ifndef STEPPER_H
#define STEPPER_H

#include <gpio.h>
#include <stdexcept>

class Stepper
{
public:
  static void init();
  static void setupTimers();
  static void setupGPIO();
  static void enable();
  static void disable();
  static void cw();
  static void ccw();
  static void step();
  static void zero();
  static void minLimit();
  static void maxLimit();
  static void setVelocity(float velocity);
  static bool isZeroed();
  static uint16_t getTicks();
  static float velocitySetpoint();

  static TIM_HandleTypeDef htim1, htim2;

private:
  static GPIO l0, l1, dm0, dm1, dm2, rst, dir, en;
  static float velocity_setpoint, max_velocity; // rev/s
  static bool zeroed, scanning;
};

#endif // STEPPER_H
