#include <stdexcept>
#include <string.h>

#include <printf.h>
#include <status.h>
#include <config.h>
#include <controller.h>

/*
 * Global Initializers
 */
TIM_HandleTypeDef Controller::htim6;
Mode Controller::mode;
GPIO Controller::led = GPIO(LED0_GPIO_Port, LED0_Pin);

// Global CAN stuff
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/*
 * Interrupt handlers
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // Get Message
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    ERROR("Reception error");
  }

  /* Display LEDx */
  uint32_t can_id = RxHeader.StdId;
  if (Controller::getID(can_id) == 1)
  {
    switch(Controller::getMessageType(can_id))
    {
    case MessageType::SetMode : // TODO add additional modes
    {
      Mode mode = (Mode)RxData[0];
      float set_point;
      memcpy(&set_point, &RxData[1], 4);
      if (!Stepper::initialized() && mode != Mode::Initialize)
      {
        mode = Mode::Disabled;
        WARN("Must initialize first");
      }
      switch (mode)
      {
      case Mode::Disabled :
        Controller::setMode(mode);
        Stepper::setScan(false);
        Stepper::disable();
        Stepper::setVelocity(0.0f);
        INFO("Disabling stepper")
        break;
      case Mode::Initialize :
        Controller::setMode(mode);
        Stepper::setScan(true);
        Stepper::enable();
        Stepper::setVelocity(set_point);
        INFO("Controller initialized")
        break;
      case Mode::Scan :
        Controller::setMode(mode);
        Stepper::setScan(true);
        Stepper::enable();
        Stepper::setVelocity(set_point);
        INFO("Starting scan")
        break;
      case Mode::Velocity :
        Controller::setMode(mode);
        Stepper::setScan(false);
        Stepper::enable();
        Stepper::setVelocity(set_point);
        INFO("[Changing to velocity mode")
        break;
      default :
        ERROR("Invalid mode");
        Controller::setMode(Mode::Disabled);
        Stepper::setScan(false);
        Stepper::disable();
        Stepper::setVelocity(0.0f);
        INFO("Invalid mode, disabling stepper")
        return;
      }
      break;
    }
    case MessageType::RequestState :
    {
      float position = Stepper::convertAngle(Stepper::getTicks());
      float velocity = Stepper::getVelocitySetPoint();
      memcpy(&TxData[0], &position, 4);
      memcpy(&TxData[4], &velocity, 4);
      CAN_TxHeaderTypeDef header;
      header.StdId = Controller::generateCanID(3, MessageType::StateMessage);
      header.ExtId = 0;
      header.RTR = CAN_RTR_DATA;
      header.IDE = CAN_ID_STD;
      header.DLC = 8;
      header.TransmitGlobalTime = DISABLE;
      if (HAL_CAN_AddTxMessage(hcan, &header, TxData, &TxMailbox) != HAL_OK)
      {
        ERROR("Unable to send CAN message");
      }
      INFO("Sent state");
      break;
    }
    case MessageType::SetPoint :
    {
      float set_point;
      memcpy(&set_point, &RxData[0], 4);
      if (Controller::getMode() == Mode::Velocity)
      {
        Stepper::setVelocity(set_point);
        INFO("Setting velocity to: %f", set_point);
      }
      else
      {
        WARN("Other control modes not implemented");
      }
      break;
    }
    default :
      ERROR("Message response not implemented");
      break;
    }
  }
}

extern "C" void TIM6_DAC1_IRQHandler()
{
  Controller::update();
}

/*
 * State machine implementation
 */
void Controller::init()
{
  mode = Mode::Disabled;
  setupTimer();
  INFO("State machine initialized");
}

void Controller::setupTimer()
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = CLOCK_SPEED_MHZ; // put quanta in us
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = CONTROL_LOOP_PERIOD_US;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    ERROR("Unable to initialize timer 6");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    ERROR("Unable to synchronize timer 6");
  }
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start_IT(&htim6);
}

uint32_t Controller::getID(uint32_t can_id)
{
  return can_id & 0b1111;
}


MessageType Controller::getMessageType(uint32_t id)
{
  return (MessageType)(id >> 4);;
}

uint32_t Controller::generateCanID(uint32_t id, MessageType type)
{
  return id | type << 4;
}

void Controller::setMode(Mode mode)
{
  Controller::mode = mode;
}

Mode Controller::getMode()
{
  return Controller::mode;
}



extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef   TxHeader;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[8];
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;
void Controller::update()
{
  //if ()
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

  switch (mode)
  {
  case Mode::Initialize :
    if (Stepper::initialized())
    {
      Controller::setMode(Mode::Disabled);
      Stepper::setScan(false);
      Stepper::disable();
      Stepper::setVelocity(0.0f);
    }
    break;
  }

  /*uint16_t ticks = Stepper::getTicks();
  memcpy(TxData, &ticks, 2);
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    ERROR("Unable to send CAN message");
  }*/
  led.toggle();

  __HAL_TIM_CLEAR_IT(&htim6, TIM6_DAC1_IRQn);
}
