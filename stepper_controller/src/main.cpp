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
//Stepper stepper;
Encoder encoder(&htim1);


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
  led.toggle();
  //stepper.step();
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
  timer.init(0xFF, 0xFFFF);
  encoder.init();
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // Initialize stepper
  /*stepper.init();
  stepper.enable();*/


  /* Infinite loop */
  while (1);
}
