#include <stdexcept>
#include <main.h>
#include <gpio.h>
#include <stepper.h>
#include <encoder.h>
#include <clock.h>
#include <service.h>

// Global variables
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
GPIO led(LD3_GPIO_Port, LD3_Pin, GPIO_MODE_OUTPUT_PP);
Service timer(&htim7, TIM7);
Encoder encoder(&htim1);
Stepper stepper;

// Function prototypes
void SystemClock_Config(void);

// Interrupts
/*extern "C" void TIM6_DAC_IRQHandler(void)
{
  //HAL_TIM_IRQHandler(&ITimer::timer6->handle);
}*/

extern "C" void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(timer.htim);
  stepper.step();
}

/*extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
}*/

extern "C" void EXTI1_IRQHandler()
{
  led.toggle();
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

}

// Program
int main(void)
{
  // Initialize hardware
  HAL_Init();
  Clock::setup();
  GPIO::setup();

  // Initialize classes
  led.init();
  timer.init(0x0, 0x2FFF);
  encoder.init();
  stepper.init();
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  // Enable stepper motor
  stepper.enable();

  /* Infinite loop */
  while (1);
}
