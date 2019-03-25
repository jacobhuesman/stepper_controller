#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <main.h>
#include <stepper.h>

  enum MessageType
  {
    Error        = 0,
    RequestState = 1,
    FindZero     = 2,
    SetZero      = 3,
    SetMode      = 4,
    SetLimits    = 5,
    SetPoint     = 6,
    StateMessage = 7, // position & velocity
  };

  enum Mode
  {
    Disabled    = 0,
    Initialize  = 1,
    Scan        = 2,
    Position    = 3,
    Velocity    = 4,
    ControlLoop = 5
  };

class Controller
{
public:
  //StateMachine(TIM_HandleTypeDef *htim6, Stepper *stepper, GPIO *led);
  static void init();
  static void update();
  static void setupTimer();
  static void setMode(Mode mode);
  static Mode getMode();

  // Helper functions
  static MessageType getMessageType(uint32_t can_id);
  static uint32_t getID(uint32_t can_id);
  static uint32_t generateCanID(uint32_t id, MessageType type);

private:
  static TIM_HandleTypeDef htim6;
  static GPIO led;
  static Mode mode;
};



#endif /* STATE_MACHINE_H */
