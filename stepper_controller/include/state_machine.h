#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <main.h>
#include <stepper.h>

enum State
{
  Disabled,
  Position,
  Velocity,
  ControlLoop
};

class StateMachine
{
public:
  //StateMachine(TIM_HandleTypeDef *htim6, Stepper *stepper, GPIO *led);
  static void init();
  static void update();
  static void setupTimer();
private:
  static TIM_HandleTypeDef htim6;
  static GPIO led;
  static State state;
};



#endif /* STATE_MACHINE_H */
