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
Stepper::Direction Stepper::active_limit;
uint32_t Stepper::limit_count;

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

void Stepper::init()
{
  // Initialize GPIO
  setupGPIO();
  disable();
  set(Stepper::cw);
  dm0.set(GPIO::high);
  dm1.set(GPIO::high);
  dm2.set(GPIO::high);
  //dm0.set(GPIO::low);
  //dm1.set(GPIO::low);
  rst.set(GPIO::low);

  // Initialize state
  zeroed = false;
  scanning = true;
  max_velocity = 0.5f;
  velocity_setpoint = 0.0f;
  limit_count = 0;
  active_limit = Direction::none;

  setupTimers();

  INFO("Stepper initialized")
}

void Stepper::enable()
{
  en.set(GPIO::high);
}

void Stepper::disable()
{
  en.set(GPIO::low);
}

void Stepper::setScan(bool value)
{
  scanning = value;
}


void Stepper::set(Direction direction)
{
  if (direction == Stepper::cw)
  {
    dir.set(GPIO::high);
  }
  else if (direction == Stepper::ccw)
  {
    dir.set(GPIO::low);
  }
}


bool Stepper::isZeroed()
{
  return zeroed;
}

void Stepper::zero()
{
  if (!zeroed)
  {
    setLimit(Stepper::ccw,-0.5f);
    setLimit(Stepper::cw,  0.5f);
    zeroed = true;
  }
  htim1.Instance->CNT = TICKS_PER_REV / 2;
}

uint16_t Stepper::getTicks()
{
  return htim1.Instance->CNT;
}

int32_t Stepper::convertVelocity(float velocity)
{
  if (velocity < 1.0e-3 && velocity > -1.0e-3)
  {
    return __INT32_MAX__;
  }
  return (int32_t)(320e3f / (velocity * 200.0f * 32.0f));

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

  if ((velocity_setpoint > 0 && active_limit == Direction::cw) || (velocity_setpoint < 0 && active_limit == Direction::ccw))
  {
    velocity_setpoint = 0.0f;
  }

  // Set
  uint32_t ivelocity_setpoint = std::abs(convertVelocity(velocity_setpoint));
  if (std::abs(velocity_setpoint) < 1e-3)
  {
    // TODO shouldn't this be zero?
    htim2.Instance->ARR =  ivelocity_setpoint;
    htim2.Instance->CCR1 = ivelocity_setpoint/2;
    htim2.Instance->CR1 = htim2.Instance->CR1 & 0xFFFE;
  }
  else
  {
    if (velocity_setpoint > 0.0f)
    {
      set(Stepper::cw);
    }
    else
    {
      set(Stepper::ccw);
    }

    active_limit = Direction::none;

    // TODO some conversion, set direction
    //htim2.Instance->CR1 = htim2.Instance->CR1 & 0xFFFE;
    htim2.Instance->CNT = 0;
    htim2.Instance->ARR =  ivelocity_setpoint;
    htim2.Instance->CCR1 = ivelocity_setpoint/2;
    htim2.Instance->CR1 = htim2.Instance->CR1 | 0x0001;
  }
}

float Stepper::convertAngle(uint16_t tics)
{
  return (float)tics / TICKS_PER_REV - 0.5f;
}
uint16_t Stepper::convertAngle(float angle)
{
  return (uint16_t)((angle + 0.5f) * TICKS_PER_REV);
}

void Stepper::setLimit(Direction direction, float angle)
{
  if (direction == Stepper::ccw)
  {
    htim1.Instance->CCR3 = convertAngle(angle);
  }
  else
  {
    htim1.Instance->CCR4 = convertAngle(angle);
  }
}

bool Stepper::initialized()
{
  return limit_count >= 2;
}

float Stepper::getVelocitySetPoint()
{
  return velocity_setpoint;
}


void Stepper::hitLimit(Direction direction)
{
  if (direction == Stepper::cw)
  {
    if (velocity_setpoint >= 1e-3)
    {
      if (scanning)
      {
        limit_count++;
        setVelocity(-velocity_setpoint);
        led2.set(GPIO::high);
      }
      else
      {
        setVelocity(0.0f);
        active_limit = direction;
      }
    }
  }
  else
  {
    if (velocity_setpoint <= -1e-3)
    {
      limit_count++;
      setVelocity(-velocity_setpoint);
      led2.set(GPIO::low);
    }
    else
    {
      setVelocity(0.0f);
      active_limit = direction;
    }
  }
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
   * - TIM2 Base Freq  = 32 MHz
   * - Max steps/rev   = 6400 stp/rev (200 bstp * 32 micstp)
   * - Prescaled base  = 320 kHz (32 MHz / 100)
   * - Max speed       = 50 rps (32 MHz / 50 / 6400 / 2)
   * - Quanta          = 1.5625e-6 us
   * - Starting rev/s  = 0.1 rev/s
   * - Starting period = 500 cc = 320 kHz / (0.1 * 6400)
   */
  {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 50;
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
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      ERROR("Unable to start PWM for timer 2");
    }
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
