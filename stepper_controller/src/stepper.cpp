#include "stepper.h"
#include "status.h"
#include <cmath>

extern Stepper stepper;
extern GPIO led2;

/*
 * Interrupt handlers
 */
extern "C" void EXTI9_5_IRQHandler()
{
  stepper.zero();
  stepper.minLimit();
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
}

extern "C" void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      stepper.maxLimit();
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      // TODO position setpoint
    }
  }
}


Stepper::Stepper(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2) :
  htim1(htim1),
  htim2(htim2),
  l0(STP_L0_GPIO_Port,   STP_L0_Pin),
  l1(STP_L1_GPIO_Port,   STP_L1_Pin),
  dm0(STP_DM0_GPIO_Port, STP_DM0_Pin),
  dm1(STP_DM1_GPIO_Port, STP_DM1_Pin),
  dm2(STP_DM2_GPIO_Port, STP_DM2_Pin),
  rst(STP_RST_GPIO_Port, STP_RST_Pin),
  dir(STP_DIR_GPIO_Port, STP_DIR_Pin),
  en(STP_EN_GPIO_Port,   STP_EN_Pin){}

void Stepper::init()
{
  // Initialize GPIO
  // TODO move initialization for the rest of stepper GPIO here
  this->disable();
  this->cw();
  dm0.set(GPIO::high);
  dm1.set(GPIO::high);
  dm2.set(GPIO::high);
  rst.set(GPIO::low);

  // Initialize state
  zeroed = false;
  scanning = true;
  max_velocity = 100.0f;
  velocity_setpoint = 100.0f;

  // Initialize index interrupt
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = ENC_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_I_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // Initialize encoder timer
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig1 = {0};
  TIM_OC_InitTypeDef sConfigOC1CH3 = {0};
  TIM_OC_InitTypeDef sConfigOC1CH4 = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  htim1->Instance = TIM1;
  htim1->Init.Prescaler = 0;
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = 8192*2;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(htim1, &sConfig) != HAL_OK)
  {
    ERROR("Unable to initialize timer 1");
  }
  sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig1.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig1) != HAL_OK)
  {
    ERROR("Unable to synchronize timer 1");
  }
  sConfigOC1CH3.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC1CH3.Pulse = 8192*3/4;
  sConfigOC1CH3.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC1CH3.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC1CH3.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC1CH3.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC1CH3.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(htim1, &sConfigOC1CH3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC1CH4.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC1CH4.Pulse = 8192*3/2;
  sConfigOC1CH4.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC1CH4.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC1CH4.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC1CH4.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC1CH4.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  /*if (HAL_TIM_OC_ConfigChannel(htim1, &sConfigOC1CH4, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }*/
  HAL_TIM_Encoder_Start(htim1, TIM_CHANNEL_ALL);
  HAL_TIM_OC_Start_IT(htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(htim1, TIM_CHANNEL_4);


  /* Initialize step timer
   * Timing
   * - TIM2 Base Freq  = 64 MHz
   * - Max steps/rev   = 6400 stp/rev (200 bstp * 32 micstp)
   * - Prescaled base  = 640 kHz (64 MHz / 100)
   * - Max speed       = 100 rps (64 MHz / 100 / 6400)
   * - Quanta          = 1.5625e-6 us
   * - Starting rev/s  = 0.1 rev/s
   * - Starting period = 2000 cc = 640 kHz / (0.1 * 6400) * 2
   */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig2 = {0};
  TIM_OC_InitTypeDef sConfigOC2 = {0};

  htim2->Instance = TIM2;
  htim2->Init.Prescaler = 100;
  htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 20;
  htim2->Init.Period = 100;
  htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim2) != HAL_OK)
  {
    ERROR("Unable to initialize timer 2");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim2, &sClockSourceConfig) != HAL_OK)
  {
    ERROR("Unable to initialize timer 2 clock source");
  }
  if (HAL_TIM_PWM_Init(htim2) != HAL_OK)
  {
    ERROR("Unable to initialize timer 2 PWM");
  }
  sMasterConfig2.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim2, &sMasterConfig2) != HAL_OK)
  {
    ERROR("Unable to synchronize timer 2");
  }
  sConfigOC2.OCMode = TIM_OCMODE_PWM1;
  sConfigOC2.Pulse = htim2->Init.Period / 2;
  sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim2, &sConfigOC2, TIM_CHANNEL_1) != HAL_OK)
  {
    ERROR("Unable to configure timer 2 PWM signal");
  }
  HAL_TIM_MspPostInit(htim2);
}

void Stepper::enable()
{
  en.set(GPIO::high);
  if (HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    ERROR("Unable to start PWM for timer 2");
  }
}

void Stepper::disable()
{
  en.set(GPIO::low);
}

void Stepper::cw()
{
  dir.set(GPIO::high);
}

void Stepper::ccw()
{
  dir.set(GPIO::low);
}

bool Stepper::isZeroed()
{
  return zeroed;
}

void Stepper::zero()
{
  htim1->Instance->CNT = 0;
}

uint16_t Stepper::getTicks()
{
  return htim1->Instance->CNT;
}

void Stepper::setVelocity(float velocity)
{
  // Threshold
  if (velocity > max_velocity)
  {
    velocity_setpoint = max_velocity;
  }
  else if (velocity < -max_velocity)
  {
    velocity_setpoint = -max_velocity;
  }
  else
  {
    velocity_setpoint = velocity;
  }

  // Set
  if (std::abs(velocity_setpoint) < 1e-6)
  {
    htim2->Instance->CR1 = htim1->Instance->CR1 & 0xFFFE;
  }
  else
  {
    // TODO some conversion, set direction
    htim2->Instance->CR1 = htim1->Instance->CR1 & 0xFFFE;
    htim2->Instance->ARR = (uint32_t) std::abs(velocity_setpoint);
    htim2->Instance->CCR1 = (uint32_t) std::abs(velocity_setpoint/2.0f);
    htim2->Instance->CNT = 0;
    htim2->Instance->CR1 = htim1->Instance->CR1 | 0x0001;
    if (velocity_setpoint > 0.0f)
    {
      this->cw();
    }
    else
    {
      this->ccw();
    }

  }
}

void Stepper::minLimit()
{
  if (stepper.scanning)
  {
    if (velocity_setpoint < 0)
    {
      setVelocity(-velocity_setpoint);
      led2.set(GPIO::low);
    }
  }
  else
  {
    setVelocity(0.0f);
  }
}

void Stepper::maxLimit()
{
  if (stepper.scanning)
  {
    if (velocity_setpoint >= 0)
    {
      setVelocity(-velocity_setpoint);
      led2.set(GPIO::high);
    }
  }
  else
  {
    setVelocity(0.0f);
  }
}
