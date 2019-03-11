#ifndef GPIO_H
#define GPIO_H

#include "main.h"

class GPIO
{
public:
  static const GPIO_PinState high = GPIO_PIN_SET;
  static const GPIO_PinState low = GPIO_PIN_RESET;

  static void init();

  GPIO() {};
  GPIO(GPIO_TypeDef* port, uint16_t pin);
  void set(GPIO_PinState value);
  void toggle();
private:
  uint16_t pin;
	GPIO_TypeDef* port;
};

#endif // GPIO_H
