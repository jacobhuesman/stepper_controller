#ifndef STEPPER_H
#define STEPPER_H

#include <gpio.h>
#include <stdexcept>

class Stepper
{
public:
  enum Direction
  {
    ccw,
    cw,
    none
  };
  static void init();
  static void setupTimers();
  static void setupGPIO();
  static void enable();
  static void disable();
  static void set(Direction direction);
  static void setLimit(Direction direction, float angle);
  static void step();
  static void zero();
  static void hitLimit(Direction direction);
  static int32_t convertVelocity(float velocity);
  static void setVelocity(float velocity);
  static bool isZeroed();
  static uint16_t getTicks();
  static float velocitySetpoint();
  static float convertAngle(uint16_t tics);
  static uint16_t convertAngle(float value);
  static float convertVelocityTics(uint16_t tics);
  static void setScan(bool value);
  static bool initialized();
  static float getVelocitySetPoint();

  static TIM_HandleTypeDef htim1, htim2;

private:
  static GPIO l0, l1, dm0, dm1, dm2, rst, dir, en;
  static float velocity_setpoint, max_velocity; // rev/s
  static bool zeroed, scanning;
  static Direction active_limit;
  static uint32_t limit_count;
};

#endif // STEPPER_H
