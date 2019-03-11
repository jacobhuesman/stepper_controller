#include "../include/main.h"
#include "stepper.h"
#define DEBUG_PRINTING 1
#include "status.h"

// HAL handles
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// Objects
Stepper stepper;
GPIO led0(LED0_GPIO_Port, LED0_Pin);
GPIO led1(LED1_GPIO_Port, LED1_Pin);
GPIO led2(LED2_GPIO_Port, LED2_Pin);

// Config functions
extern "C" void MX_GPIO_Init(void);
extern "C" void MX_USART2_UART_Init(void);
extern "C" void MX_RTC_Init(void);
extern "C" void MX_USART1_UART_Init(void);
extern "C" void MX_ADC1_Init(void);
extern "C" int iprintf(const char *fmt, ...);

// can.c
extern "C" void MX_CAN_Init(void);

// clocks.c
extern "C" void SystemClock_Config(void);

// timers.c
extern "C" void MX_TIM1_Init(void);
extern "C" void MX_TIM2_Init(void);
extern "C" void MX_TIM6_Init(void);
extern "C" void MX_TIM16_Init(void);

// Temp CAN stuff
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

int main(void)

{
  printf("....\n"); // Sacrifice some periods to the gods of SWD
  HAL_Init();
  SystemClock_Config();
  GPIO::init();
  //MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_RTC_Init();
  //MX_USART1_UART_Init();
  INFO("Initialized System");

  // Initialize objects
  stepper.init();
  stepper.enable();
  stepper.ccw();

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  TxData[0] = 0x12;
  TxData[1] = 0x00;


  while (1);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // Get Message
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  led1.toggle();


  /* Display LEDx */
  if ((RxHeader.StdId == 0x1) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
    //LED_Display(RxData[0]);
    //ubKeyNumber = RxData[0];
  }
}

// Just toggling for now
extern "C" void EXTI9_5_IRQHandler()
{
  led2.toggle();
  htim1.Instance->CNT = 4096;
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_5);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
}

// Just toggling for now
uint32_t count = 0;
extern "C" void tim16_it_handler(void)
{
  //led0.toggle();
}

// High precision toggling...
// TODO calibrate HSI instead of counting LSI cycles
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (count++ == 790)
  {
    TxData[1]++;
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      Error_Handler();
    }
    led0.toggle();
    count = 0;
  }
}


/*extern "C" void TIM6_DAC1_IRQHandler()
{
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  TxData[1]++;
  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_IT(&htim6, TIM6_DAC1_IRQn);
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

