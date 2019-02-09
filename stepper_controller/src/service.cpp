#include <service.h>
#include <stdexcept>


Service::Service(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
  this->htim = htim;
	this->htim->Instance = instance;
}

#define Error_Handler() while(1)

void Service::init(uint32_t prescaler, uint32_t period)
{
  // Setup timer
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim->Instance = TIM7;
  htim->Init.Prescaler = prescaler;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = period;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(htim) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // Enable timer
  HAL_TIM_Base_Start(htim);
  HAL_TIM_Base_Start_IT(htim);
  /*handle->Instance->CR1 |= TIM_CR1_CEN;
  handle->Instance->DIER |= TIM_DIER_UIE;*/
}
