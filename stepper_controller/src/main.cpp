#include <controller.h>
#include <status.h>
#include <main.h>
#include <stepper.h>
#include <test.h>
#include <tests/stepper_tests.h>
#include <tests/sanity_tests.h>

// HAL handles
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
RTC_HandleTypeDef hrtc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// Global Objects
GPIO led1(LED1_GPIO_Port, LED1_Pin);
GPIO led2(LED2_GPIO_Port, LED2_Pin);
//StateMachine state(&htim6, &stepper, &led0);

// Helper functions
extern "C" void MX_USART1_UART_Init(void);     // uart.c
extern "C" void MX_USART2_UART_Init(void);     // uart.c
extern "C" void MX_CAN_Init(void);             // can.c
extern "C" void SystemClock_Config(void);      // clocks.c
void errorHandler(std::exception &e);

void _putchar(char character)
{
  ITM_SendChar(character);
}
/*
 *
 *
 *
 *
 *
 *
 *
 * JUST IMPLEMENT SET VELOCITY, GET POSITION AND GET VELOCITY OVER CAN AND WRITE CONTROL SYSTEM ON COMPUTER END
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */


int main(void)
{
  printf("....\n"); // Sacrifice some periods to the gods of SWD
  HAL_Init();
  SystemClock_Config();
  INFO("Initialized system");

  Test::runAll();
  INFO("Ran tests");

  MX_CAN_Init();
  GPIO::init();
  Stepper::init();
  Controller::init();
  INFO("Initialized controller");

  while (1);
}

/*
 * Callbacks
 */


// Default CubeMx error handler
void Error_Handler()
{
  ERROR("Something bad happened");
}


