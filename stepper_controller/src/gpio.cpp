#include <gpio.h>

GPIO::GPIO(GPIO_TypeDef* port, uint16_t pin)
{
  this->pin = pin;
  this->port = port;
}

void GPIO::set(GPIO_PinState value)
{
  HAL_GPIO_WritePin(this->port, this->pin, value);
}

void GPIO::toggle()
{
	HAL_GPIO_TogglePin(this->port, this->pin);
}
