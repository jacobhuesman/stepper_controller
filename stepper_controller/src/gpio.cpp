#include <gpio.h>
#include <stm32l4xx_hal.h>

void GPIO::setup()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
}

GPIO::GPIO(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t speed, uint32_t pull)
{
  config = {0};
  config.Pin = pin;
  config.Mode = mode;
  config.Pull = pull;
  config.Speed = speed;
  this->port = port;
}

void GPIO::init()
{
  HAL_GPIO_Init(port, &config);
}

void GPIO::on()
{
  this->set(true);
}

void GPIO::off()
{
  this->set(false);
}

void GPIO::setLow()
{
  this->set(true);
}

void GPIO::setHigh()
{
  this->set(false);
}

void GPIO::set(bool value)
{
	if (value)
	{
		HAL_GPIO_WritePin(port, config.Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(port, config.Pin, GPIO_PIN_SET);
	}
}

void GPIO::toggle()
{
	HAL_GPIO_TogglePin(port, config.Pin);
}
