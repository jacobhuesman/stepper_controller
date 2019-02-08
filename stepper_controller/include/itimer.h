#ifndef ITIMER_H
#define ITIMER_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim_ex.h"


class ITimer
{
public:
  ITimer(TIM_TypeDef *instance);
	void init(uint32_t prescaler,uint32_t period);
	TIM_HandleTypeDef handle;
};

void MX_TIM6_Init(void);
void MX_TIM7_Init(void);

#endif // ITIMER_H
