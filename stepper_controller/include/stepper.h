#ifndef STEPPER_H
#define STEPPER_H

#include <gpio.h>

class Stepper
{
public:
  Stepper();
  void init();
  void enable();
  void disable();
  void cw();
  void ccw();
  void step();
private:
  GPIO l0, l1, dm0, dm1, dm2, rst, dir, en;
};

#endif // STEPPER_H
