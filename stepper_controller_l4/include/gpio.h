#ifndef GPIO_H
#define GPIO_H

#include "stm32l4xx_hal_gpio.h"

class GPIO
{
public:
  static void setup();
  GPIO() {};
  GPIO(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t speed = GPIO_SPEED_FREQ_LOW, uint32_t pull = GPIO_NOPULL);
  void init();
  void setLow();
  void setHigh();
  void on();
	void off();
  void set(bool value);
	void toggle();
private:
	GPIO_InitTypeDef config;
	GPIO_TypeDef* port;
};

#endif // GPIO_H
