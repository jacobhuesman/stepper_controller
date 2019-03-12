#include "stepper.h"

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
  this->disable();
  this->cw();
  dm0.set(GPIO::high);
  dm1.set(GPIO::high);
  dm2.set(GPIO::high);
  rst.set(GPIO::low);

  // Initialize state
  zeroed = false;
  max_velocity = 1.0f;

  // Initialize encoder timer
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig1 = {0};
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
    throw std::runtime_error("Unable to initialize timer 1");
  }
  sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig1.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig1) != HAL_OK)
  {
    throw std::runtime_error("Unable to synchronize timer 1");
  }
  HAL_TIM_Encoder_Start(htim1, TIM_CHANNEL_ALL);


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
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2->Instance = TIM2;
  htim2->Init.Prescaler = 100;
  htim2->Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 20;
  htim2->Init.Period = 1000;
  htim2->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim2) != HAL_OK)
  {
    throw std::runtime_error("Unable to initialize timer 2");
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(htim2, &sClockSourceConfig) != HAL_OK)
  {
    throw std::runtime_error("Unable to initialize timer 2 clock source");
  }
  if (HAL_TIM_PWM_Init(htim2) != HAL_OK)
  {
    throw std::runtime_error("Unable to initialize timer 2 PWM");
  }
  sMasterConfig2.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig2.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim2, &sMasterConfig2) != HAL_OK)
  {
    throw std::runtime_error("Unable to synchronize timer 2");
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim2->Init.Period / 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    throw std::runtime_error("Unable to configure timer 2 PWM signal");
  }
  HAL_TIM_MspPostInit(htim2);
}

void Stepper::enable()
{
  en.set(GPIO::high);
  if (HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
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
