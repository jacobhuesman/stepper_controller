#include "stepper.h"

#define STP_FLT_Pin GPIO_PIN_0
#define STP_FLT_GPIO_Port GPIOA
#define STP_EN_Pin GPIO_PIN_1
#define STP_EN_GPIO_Port GPIOA
#define STP_RST_Pin GPIO_PIN_2
#define STP_RST_GPIO_Port GPIOA
#define STP_DIR_Pin GPIO_PIN_3
#define STP_DIR_GPIO_Port GPIOA
#define STP_STEP_Pin GPIO_PIN_4
#define STP_STEP_GPIO_Port GPIOA
#define STP_DM0_Pin GPIO_PIN_5
#define STP_DM0_GPIO_Port GPIOA
#define STP_DM1_Pin GPIO_PIN_6
#define STP_DM1_GPIO_Port GPIOA
#define STP_DM2_Pin GPIO_PIN_7
#define STP_DM2_GPIO_Port GPIOA

/*
  GPIO_InitStruct.Pin = STP_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STP_STEP_GPIO_Port, &GPIO_InitStruct);
*/
Stepper::Stepper() :
  flt(STP_FLT_GPIO_Port,  STP_FLT_Pin,  GPIO_MODE_INPUT),
  dm0(STP_DM0_GPIO_Port,  STP_DM0_Pin,  GPIO_MODE_OUTPUT_PP),
  dm1(STP_DM1_GPIO_Port,  STP_DM1_Pin,  GPIO_MODE_OUTPUT_PP),
  dm2(STP_DM2_GPIO_Port,  STP_DM2_Pin,  GPIO_MODE_OUTPUT_PP),
  en(STP_EN_GPIO_Port,    STP_EN_Pin,   GPIO_MODE_OUTPUT_PP),
  rst(STP_RST_GPIO_Port,  STP_RST_Pin,  GPIO_MODE_OUTPUT_PP),
  dir(STP_DIR_GPIO_Port,  STP_DIR_Pin,  GPIO_MODE_OUTPUT_PP),
  stp(STP_STEP_GPIO_Port, STP_STEP_Pin, GPIO_MODE_OUTPUT_PP)
{

}

void Stepper::init()
{
  // Initialize pins
  flt.init();
  dm0.init();
  dm1.init();
  dm2.init();
  en.init();
  rst.init();
  dir.init();
  stp.init();

  // Set values
  dm0.setHigh();
  dm1.setHigh();
  dm2.setHigh();
  rst.setHigh();
  en.setLow();
  cw();
}

void Stepper::enable()
{
  en.setLow();
}

void Stepper::disable()
{
  en.setHigh();
}

void Stepper::cw()
{
  dir.setHigh();
}

void Stepper::ccw()
{
  dir.setLow();
}

void Stepper::step()
{
  stp.toggle();
}
