#ifndef GPIO_H
#define GPIO_H

#include "stm32l4xx_hal_gpio.h"

// IO Defines
#define ENC_B_Pin GPIO_PIN_0
#define ENC_B_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define ENC_X_Pin GPIO_PIN_6
#define ENC_X_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_7
#define ENC_A_GPIO_Port GPIOB

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
