/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "main.h"
#include "stdlib.h"
#include "time.h"

#define led1 GPIO_PIN_0
#define led2 GPIO_PIN_1
#define led3 GPIO_PIN_2
#define led4 GPIO_PIN_3
#define led5 GPIO_PIN_4
#define led6 GPIO_PIN_5
#define led7 GPIO_PIN_6

int LEDS[]= {led1,led2,led3,led4,led5,led6,led7};
#define button GPIO_PIN_13


void SystemClock_Config(void);
static void MX_GPIO_Init(void);

//
// This function turns OFF all LEDS
//
void ALL_OFF()
{
	int i;
	for(i = 0; i < 7; i++)
	{
		HAL_GPIO_WritePin(GPIOC, LEDS[i], GPIO_PIN_RESET);
	}
}


int main(void)
{
  int dice;
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  srand(time(NULL));												// Random number seed
  while (1)															// DO FOREVER
  {
	  ALL_OFF();
	  while(HAL_GPIO_ReadPin(GPIOC, button) == 1);					// Wait for button
	  dice = rand()%6 + 1;											// Get a dice number

	  switch(dice)
	  {
	  case 1:														// IS it 1?
		  HAL_GPIO_WritePin(GPIOC,LEDS[3], GPIO_PIN_SET);
		  break;
	  case 2:														// Is it 2?
		  HAL_GPIO_WritePin(GPIOC,LEDS[1], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[5], GPIO_PIN_SET);
		  break;
	  case 3:														// Is it 3?
		  HAL_GPIO_WritePin(GPIOC,LEDS[1], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[3], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[5], GPIO_PIN_SET);
		  break;
	  case 4:														// Is it 4?
		  HAL_GPIO_WritePin(GPIOC,LEDS[0], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[2], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[4], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[6], GPIO_PIN_SET);
		  break;
	  case 5:														// IS it 5?
		  HAL_GPIO_WritePin(GPIOC,LEDS[0], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[2], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[3], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[4], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[6], GPIO_PIN_SET);
		  break;
	  case 6:														// Is it 6?
		  HAL_GPIO_WritePin(GPIOC,LEDS[0], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[1], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[2], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[4], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[5], GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,LEDS[6], GPIO_PIN_SET);
		  break;
	  }
	  HAL_Delay(3000);												// Wait 3 seconds
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{ 
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
