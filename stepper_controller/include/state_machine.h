#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"
#include "stepper.h"


class StateMachine
{
public:
  StateMachine(TIM_HandleTypeDef *htim6, Stepper *stepper, GPIO *led);
  void init();
  void update();
private:
  TIM_HandleTypeDef *htim6;
  Stepper *stepper;
  GPIO *led;
};



#endif /* STATE_MACHINE_H */
