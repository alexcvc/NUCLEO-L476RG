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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"

//
// Define Thread IDs
//
osThreadId_t LEDSTaskHandle;								// Slow LED
osThreadId_t LEDMTaskHandle;								// Medium LED
osThreadId_t LEDFTaskHandle;								// Fast LED

//
// Slow LED task. Flash every second
//
void StartLEDSTask(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, LEDSLOW_Pin);
    osDelay(1000);
  }
}

//
// Medium LED task. Flash every 500ms
//
void StartLEDMTask(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, LEDMEDIUM_Pin);
    osDelay(500);
  }
}

//
// Fast LED task. Flash every 250ms
//
void StartLEDFTask(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOC, LEDFAST_Pin);
        osDelay(250);
  }
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

//
// Start of main program
//
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

//
// Slow LED Task attributes
//
const osThreadAttr_t LEDSTask_attributes =
{
    .name = "LEDSTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};

//
// Medium LED Task attributes
//
const osThreadAttr_t LEDMTask_attributes =
{
    .name = "LEDMTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};

//
// Fast LED Task attributes
//
const osThreadAttr_t LEDFTask_attributes =
{
    .name = "LEDFTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};

/* Init scheduler */
  osKernelInitialize();

/* creation of Tasks */
  LEDSTaskHandle = osThreadNew(StartLEDSTask, NULL, &LEDSTask_attributes);
  LEDMTaskHandle = osThreadNew(StartLEDMTask, NULL, &LEDMTask_attributes);
  LEDFTaskHandle = osThreadNew(StartLEDFTask, NULL, &LEDFTask_attributes);

  /* Start scheduler */
  osKernelStart();

  while (1)
  {
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
  HAL_GPIO_WritePin(GPIOC, LEDSLOW_Pin|LEDMEDIUM_Pin|LEDFAST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDSLOW_Pin LEDMEDIUM_Pin LEDFAST_Pin */
  GPIO_InitStruct.Pin = LEDSLOW_Pin|LEDMEDIUM_Pin|LEDFAST_Pin;
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
