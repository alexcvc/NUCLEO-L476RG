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

#define digit1 GPIO_PIN_7
#define digit2 GPIO_PIN_8

int LEDS[] = {0x3F,    /* 0 */
		     0x06,     /* 1 */
			 0x5B,     /* 2 */
			 0x4F,     /* 3 */
			 0x66,     /* 4 */
			 0x6D,     /* 5 */
			 0x7D,     /* 6 */
			 0x07,     /* 7 */
			 0x7F,     /* 8 */
			 0x6F      /* 9 */
             };

int Count = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

//
// This is the Interrupt Service Routine. The program jumps here when the
// button is pressed
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	Count++;
	if(Count == 100)Count = 0;
}

//
// Start of main program loop
//
int main(void)
{
  int MSD, LSD;
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  HAL_GPIO_WritePin(GPIOC, digit1, GPIO_PIN_RESET);			// Clear digit1
  HAL_GPIO_WritePin(GPIOC, digit2, GPIO_PIN_RESET);			// Clear digit2

  while (1)
  {
	  MSD = Count / 10;										// Extract MSD digit
	  LSD = Count % 10;										// Extract LSD digit
	  if(MSD != 0)											// If MSD is non zero
	  {
		  GPIOC -> ODR = LEDS[MSD];							// Send MSD to display
		  HAL_GPIO_WritePin(GPIOC,digit1,GPIO_PIN_SET);		// Enable digit1
		  HAL_Delay(10);									// Wait 10ms
		  HAL_GPIO_WritePin(GPIOC,digit1,GPIO_PIN_RESET);	// Disable digit1
	  }

	  GPIOC -> ODR = LEDS[LSD];								// Send LSD digit
	  HAL_GPIO_WritePin(GPIOC,digit2,GPIO_PIN_SET);			// Enable digit2
	  HAL_Delay(10);										// Wait 10ms
	  HAL_GPIO_WritePin(GPIOC,digit2,GPIO_PIN_RESET);		// Disable digit2
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
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 PC7 
                           PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
