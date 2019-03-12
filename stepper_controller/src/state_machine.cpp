#include <state_machine.h>
#include <stdexcept>
#include <string.h>
#include <status.h>

/*
 * Interrupt handler
 */
extern StateMachine state;
extern "C" void TIM6_DAC1_IRQHandler()
{
  state.update();
}

/*
 * State machine implementation
 */
StateMachine::StateMachine(TIM_HandleTypeDef *htim6, Stepper *stepper, GPIO *led) :
  htim6(htim6),
  stepper(stepper),
  led(led){}

void StateMachine::init()
{
  // Initialize timer 6
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim6->Instance = TIM6;
  htim6->Init.Prescaler = 64; // put quanta in us
  htim6->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6->Init.Period = 20000 - 350; // 50 Hz = 1 / 20 ms
  htim6->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim6) != HAL_OK)
  {
    ERROR("Unable to initialize timer 6");
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim6, &sMasterConfig) != HAL_OK)
  {
    ERROR("Unable to synchronize timer 6");
  }
  HAL_TIM_Base_Start(htim6);
  HAL_TIM_Base_Start_IT(htim6);
}

extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef   TxHeader;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               TxData[8];
extern uint8_t               RxData[8];
extern uint32_t              TxMailbox;
void StateMachine::update()
{
  __HAL_TIM_CLEAR_FLAG(htim6, TIM_FLAG_UPDATE);

  uint16_t ticks = state.stepper->getTicks();
  memcpy(TxData, &ticks, 2);
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    ERROR("Unable to send CAN message");
  }
  led->toggle();

  __HAL_TIM_CLEAR_IT(htim6, TIM6_DAC1_IRQn);
}
