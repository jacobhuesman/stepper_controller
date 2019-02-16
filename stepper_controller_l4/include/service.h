#ifndef SERVICE_H
#define SERVICE_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim_ex.h"

class Service
{
public:
  Service(TIM_HandleTypeDef *handle, TIM_TypeDef *instance);
	void init(uint32_t prescaler,uint32_t period);
	TIM_HandleTypeDef *htim;
};

#endif // SERVICE_H
