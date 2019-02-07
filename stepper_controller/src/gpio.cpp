#include <gpio.h>
#include <stm32l4xx_hal.h>


void GPIO::initialize()
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
/*
	  HAL_GPIO_WritePin(GPIOA, STP_EN_Pin|STP_RST_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOA, STP_DIR_Pin|STP_STEP_Pin|STP_DM0_Pin|STP_DM1_Pin
	                          |STP_DM2_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	  GPIO_InitStruct.Pin = STP_FLT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(STP_FLT_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = STP_EN_Pin|STP_RST_Pin|STP_DIR_Pin|STP_DM0_Pin
	                          |STP_DM1_Pin|STP_DM2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = STP_STEP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(STP_STEP_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = ENC_B_Pin|ENC_X_Pin|ENC_A_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);*/
}

GPIO::GPIO(GPIO_TypeDef* port, uint16_t pin, uint32_t mode, uint32_t speed, uint32_t pull)
{
  GPIO_InitTypeDef init = {0};
  init.Pin = pin;
  init.Mode = mode;
  init.Pull = pull;
  init.Speed = speed;
  HAL_GPIO_Init(port, &init);
  this->port = port;
  this->pin = pin;
}

void GPIO::on()
{
  this->set(true);
}

void GPIO::off()
{
  this->set(false);
}

void GPIO::set(bool value)
{
	if (value)
	{
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	}
}

void GPIO::toggle()
{
	HAL_GPIO_TogglePin(port, pin);
}

