#include <itimer.h>
#include <stdexcept>
#include "stm32l4xx.h"
#include "gpio.h"
#include <stepper.h>

// Global variables
CAN_HandleTypeDef hcan1;
GPIO led(LED_GPIO_Port, LED_Pin, GPIO_MODE_OUTPUT_PP);
ITimer tim7(TIM7);
Stepper stepper;

// Function prototypes
void SystemClock_Config(void);

// Interrupts
extern "C" void TIM6_DAC_IRQHandler(void)
{
  //HAL_TIM_IRQHandler(&ITimer::timer6->handle);
}

extern "C" void TIM7_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim7.handle);
  led.toggle();
  stepper.step();
}

// Program
int main(void)
{
  // Initialize hardware
  SystemClock_Config();
  HAL_Init();
  GPIO::setup();
  led.init();
  tim7.init(0, 255);
  stepper.init();
  stepper.enable();


  /* Infinite loop */
  while (1);
}

// Function implementations
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    throw(std::runtime_error("Oscillator config failed"));
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    throw(std::runtime_error("Clock config failed"));
  }
  /**Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    throw(std::runtime_error("Voltage scaling failed"));
  }
}
