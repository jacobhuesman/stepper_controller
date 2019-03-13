#include <stepper.h>
#include <status.h>
#include <config.h>
#include <cmath>

extern GPIO led2;

TIM_HandleTypeDef Stepper::htim1;
TIM_HandleTypeDef Stepper::htim2;
GPIO Stepper::l0  = GPIO(STP_L0_GPIO_Port,  STP_L0_Pin);
GPIO Stepper::l1  = GPIO(STP_L1_GPIO_Port,  STP_L1_Pin);
GPIO Stepper::dm0 = GPIO(STP_DM0_GPIO_Port, STP_DM0_Pin);
GPIO Stepper::dm1 = GPIO(STP_DM1_GPIO_Port, STP_DM1_Pin);
GPIO Stepper::dm2 = GPIO(STP_DM2_GPIO_Port, STP_DM2_Pin);
GPIO Stepper::rst = GPIO(STP_RST_GPIO_Port, STP_RST_Pin);
GPIO Stepper::dir = GPIO(STP_DIR_GPIO_Port, STP_DIR_Pin);
GPIO Stepper::en  = GPIO(STP_EN_GPIO_Port,  STP_EN_Pin);
float Stepper::max_velocity;
float Stepper::velocity_setpoint;
bool Stepper::zeroed;
bool Stepper::scanning;

void Stepper::init()
{
  // Initialize GPIO
  setupGPIO();
  disable();
  cw();
  dm0.set(GPIO::high);
  dm1.set(GPIO::high);
  dm2.set(GPIO::high);
  rst.set(GPIO::low);

  // Initialize state
  zeroed = false;
  scanning = true;
  max_velocity = 100.0f;
  velocity_setpoint = 100.0f;

  setupTimers();

  INFO("Stepper initialized")
}

void Stepper::setupTimers()
{
  // Initialize encoder timer
  {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = TICKS_PER_REV*2;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
    {
      ERROR("Unable to initialize timer 1");
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
      ERROR("Unable to synchronize timer 1");
    }
    // Common output capture settings
    sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    // Output capture channel 3
    sConfigOC.Pulse = TICKS_PER_REV/2;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      ERROR("Unable to configure timer 1 output capture channel 3");
    }
    // Output capture channel 4
    sConfigOC.Pulse = TICKS_PER_REV*3/2;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CNT = TICKS_PER_REV;
  }

  /*
   * Step timer
   * - TIM2 Base Freq  = 64 MHz
   * - Max steps/rev   = 6400 stp/rev (200 bstp * 32 micstp)
   * - Prescaled base  = 640 kHz (64 MHz / 100)
   * - Max speed       = 100 rps (64 MHz / 100 / 6400)
   * - Quanta          = 1.5625e-6 us
   * - Starting rev/s  = 0.1 rev/s
   * - Starting period = 2000 cc = 640 kHz / (0.1 * 6400) * 2
   */
  {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 100;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    //htim2.Init.Period = 20;
    htim2.Init.Period = 100;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      ERROR("Unable to initialize timer 2");
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
      ERROR("Unable to initialize timer 2 clock source");
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
      ERROR("Unable to initialize timer 2 PWM");
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
      ERROR("Unable to synchronize timer 2");
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = htim2.Init.Period / 2;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      ERROR("Unable to configure timer 2 PWM signal");
    }
    HAL_TIM_MspPostInit(&htim2);
  }
}

void Stepper::setupGPIO()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Initialize index interrupt
  GPIO_InitStruct.Pin = ENC_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_I_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // Output pins
  HAL_GPIO_WritePin(GPIOA, STP_DM0_Pin|STP_DM1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, STP_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, STP_DM2_Pin|STP_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, STP_RST_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = STP_EN_Pin|STP_DM0_Pin|STP_DM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = STP_DM2_Pin|STP_DIR_Pin|STP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Input pins
  GPIO_InitStruct.Pin = STP_L0_Pin|STP_L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Stepper::enable()
{
  en.set(GPIO::high);
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
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
  if (!zeroed)
  {
    htim1.Instance->CCR3 = TICKS_PER_REV/8;
    htim1.Instance->CCR4 = TICKS_PER_REV;
    zeroed = true;
  }
  htim1.Instance->CNT = TICKS_PER_REV / 2;
}

uint16_t Stepper::getTicks()
{
  return htim1.Instance->CNT;
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
    htim2.Instance->CR1 = htim2.Instance->CR1 & 0xFFFE;
  }
  else
  {
    // TODO some conversion, set direction
    htim2.Instance->CR1 = htim2.Instance->CR1 & 0xFFFE;
    htim2.Instance->ARR = (uint32_t) std::abs(velocity_setpoint);
    htim2.Instance->CCR1 = (uint32_t) std::abs(velocity_setpoint/2.0f);
    htim2.Instance->CNT = 0;
    htim2.Instance->CR1 = htim2.Instance->CR1 | 0x0001;
    if (velocity_setpoint > 0.0f)
    {
      cw();
    }
    else
    {
      ccw();
    }

  }
}

void Stepper::ccwLimit()
{
  if (scanning)
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

void Stepper::cwLimit()
{
  if (scanning)
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
