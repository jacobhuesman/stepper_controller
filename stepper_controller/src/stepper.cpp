#include "stepper.h"

Stepper::Stepper() :
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
  this->disable();
  this->cw();
  dm0.set(GPIO::high);
  dm1.set(GPIO::high);
  dm2.set(GPIO::high);
  rst.set(GPIO::high);
}

void Stepper::enable()
{
  en.set(GPIO::low);
}

void Stepper::disable()
{
  en.set(GPIO::high);
}

void Stepper::cw()
{
  dir.set(GPIO::high);
}

void Stepper::ccw()
{
  dir.set(GPIO::low);
}
