#include "encoder.h"
#include "main.h"
#include <stdexcept>

Encoder::Encoder(TIM_HandleTypeDef *htim1) :
  index(ENC_IND_GPIO_Port, ENC_IND_Pin, GPIO_MODE_IT_RISING)
{
  this->htim1 = htim1;
}

#define Error_Handler() while(1)

void Encoder::init()
{
  // Configure pulse counting
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1->Instance = TIM1;
  htim1->Init.Prescaler = 0;
  htim1->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1->Init.Period = 8192;
  htim1->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1->Init.RepetitionCounter = 0;
  htim1->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable
  index.init();
  HAL_TIM_Encoder_Start(htim1, TIM_CHANNEL_ALL);
}
