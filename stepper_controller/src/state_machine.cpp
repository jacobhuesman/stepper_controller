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
GPIO    StateMachine::led     = GPIO(LED0_GPIO_Port, LED0_Pin);

/*
 * Interrupt handlers
 */
extern "C" void EXTI9_5_IRQHandler()
{
  Stepper::zero();
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
}

extern "C" void TIM1_CC_IRQHandler(void)
{
  // Channel 3
  if(__HAL_TIM_GET_FLAG(&Stepper::htim1, TIM_FLAG_CC3) != RESET && __HAL_TIM_GET_IT_SOURCE(&Stepper::htim1, TIM_IT_CC3) != RESET)
  {
    Stepper::hitLimit(Stepper::ccw);
    __HAL_TIM_CLEAR_FLAG(&Stepper::htim1, TIM_FLAG_CC3);
  }
  // Channel 4
  if(__HAL_TIM_GET_FLAG(&Stepper::htim1, TIM_FLAG_CC4) != RESET && __HAL_TIM_GET_IT_SOURCE(&Stepper::htim1, TIM_IT_CC4) != RESET)
  {
    Stepper::hitLimit(Stepper::cw);
    __HAL_TIM_CLEAR_FLAG(&Stepper::htim1, TIM_FLAG_CC4);
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
  //Stepper::setVelocity(-100);
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
