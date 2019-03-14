#include <stdexcept>
#include <string.h>

#include <state_machine.h>
#include <status.h>
#include <config.h>

/*
 * Global Initializers
 */
TIM_HandleTypeDef StateMachine::htim6;
State   StateMachine::state;
GPIO    StateMachine::led = GPIO(LED0_GPIO_Port, LED0_Pin);

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
  if ((RxHeader.StdId == 0x1) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {

  }
}

extern "C" void TIM6_DAC1_IRQHandler()
{
  StateMachine::update();
}

/*
 * State machine implementation
 */
void StateMachine::init()
{
  state = State::Disabled;
  Stepper::enable();
  Stepper::setVelocity(0.2);
  //setupTimer();
  INFO("State machine initialized");
}

void StateMachine::setupTimer()
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

extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef   TxHeader;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[8];
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;
void StateMachine::update()
{
  //if ()
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

  uint16_t ticks = Stepper::getTicks();
  memcpy(TxData, &ticks, 2);
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    ERROR("Unable to send CAN message");
  }
  led.toggle();

  __HAL_TIM_CLEAR_IT(&htim6, TIM6_DAC1_IRQn);
}
