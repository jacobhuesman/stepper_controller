#include "stepper.h"
#include "main.h"


Stepper::Stepper() :
  flt(STP_FLT_GPIO_Port, STP_FLT_Pin, GPIO_MODE_INPUT),
  dm0(STP_DM0_GPIO_Port, STP_DM0_Pin, GPIO_MODE_OUTPUT_PP),
  dm1(STP_DM1_GPIO_Port, STP_DM1_Pin, GPIO_MODE_OUTPUT_PP),
  dm2(STP_DM2_GPIO_Port, STP_DM2_Pin, GPIO_MODE_OUTPUT_PP),
  rst(STP_RST_GPIO_Port, STP_RST_Pin, GPIO_MODE_OUTPUT_PP),
  dir(STP_DIR_GPIO_Port, STP_DIR_Pin, GPIO_MODE_OUTPUT_PP),
  stp(STP_STP_GPIO_Port, STP_STP_Pin, GPIO_MODE_OUTPUT_PP),
  en(STP_EN_GPIO_Port,   STP_EN_Pin,  GPIO_MODE_OUTPUT_PP){}

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
