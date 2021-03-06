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
  GPIO flt, dm0, dm1, dm2, rst, dir, stp, en;
};

#endif // STEPPER_H
