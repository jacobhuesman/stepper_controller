#include "stm32l4xx_hal_gpio.h"

// IO Defines
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
  static void initialize();
  GPIO(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t speed = GPIO_SPEED_FREQ_LOW, uint32_t pull = GPIO_NOPULL);
	void on();
	void off();
  void set(bool value);
	void toggle();
private:
	GPIO_TypeDef* port;
	uint16_t pin;
};
