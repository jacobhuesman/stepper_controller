#include <status.h>
#include <main.h>
#include <stepper.h>
#include <state_machine.h>

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

// Global CAN stuff
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

// Helper functions
extern "C" void MX_USART1_UART_Init(void);     // uart.c
extern "C" void MX_USART2_UART_Init(void);     // uart.c
extern "C" void MX_CAN_Init(void);             // can.c
extern "C" void SystemClock_Config(void);      // clocks.c
void errorHandler(std::exception &e);

int main(void)

{
  // Initialize system
  printf("....\n"); // Sacrifice some periods to the gods of SWD
  HAL_Init();
  SystemClock_Config();
  MX_CAN_Init();
  GPIO::init();
  Stepper::init();
  StateMachine::init();
  INFO("Initialized system");


  // Start stepper
  /*
  stepper.cw();
  stepper.enable();
  stepper.setVelocity(-100);
  */

  while (1);
}

/*
 * Callbacks
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  // Get Message
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    ERROR("Reception error");
  }
  led1.toggle();
  //stepper.setVelocity(100);


  /* Display LEDx */
  if ((RxHeader.StdId == 0x1) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
    //LED_Display(RxData[0]);
    //ubKeyNumber = RxData[0];
  }
}

// Default CubeMx error handler
void Error_Handler()
{
  ERROR("Something bad happened");
}


