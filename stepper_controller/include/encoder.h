#ifndef ENCODER_H
#define ENCODER_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim_ex.h"

class Encoder
{
public:
  Encoder(TIM_HandleTypeDef *htim1);
  void init();
  TIM_HandleTypeDef *htim1;
};

#endif // ENCODER_H
