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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

//
// Define LPS22HW register addresses
//
#define LPS22HBW 0xBA
#define LPS22HBR 0xBB
#define CTRL_REG1 0x10
#define CTRL_REG2 0x11
#define FIFO_CTRL 0x13
#define STATUS 0x27

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

int tempC;
uint8_t Dat[2];

//
// This function returns the LPS22HB status
//
uint8_t LPS22HB_GetStatus(void)
{
	uint8_t byte;
	byte=0x27;
    HAL_I2C_Master_Transmit(&hi2c1, LPS22HBW, &byte, 1, 500);
    HAL_I2C_Master_Receive(&hi2c1, LPS22HBR, Dat,1, 500);
   	return Dat[0];

}

//
// This function initializes the LPS22HB
//
void LPS22HB_Init(void)
{
    	Dat[0] = CTRL_REG1;
    	Dat[1] = 0x3A;
    	HAL_I2C_Master_Transmit(&hi2c1, LPS22HBW, Dat, 2, 500);

    	Dat[0] = CTRL_REG2;
    	Dat[1] = 0x10;
    	HAL_I2C_Master_Transmit(&hi2c1, LPS22HBW, Dat, 2, 500);

    	Dat[0] = FIFO_CTRL;
    	Dat[1] = 0x02;
    	HAL_I2C_Master_Transmit(&hi2c1, LPS22HBW, Dat, 2, 500);
    }

//
// This function returns the temperature
//
int LPS22HB_GetTemperature(void)
{
    	uint16_t temperature;
    	int temp;

    	while ((LPS22HB_GetStatus() & 2) == 0)
           HAL_Delay(100);

    	Dat[0] = 0x2B;
    	HAL_I2C_Master_Transmit(&hi2c1, LPS22HBW, Dat, 1, 500);
    	HAL_I2C_Master_Receive(&hi2c1, LPS22HBR, Dat,2,500);
    	temperature = (((uint16_t)Dat[1]) << 8) | (uint16_t) Dat[0];
    	temp = temperature / 100.0;
    	return temp;
   }


//
// Send a newline to PC
//
void UART_SEND_NL(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(huart, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

//
// Send an integer number to PC
//
void UART_SEND_INT(UART_HandleTypeDef *huart, int i, int m)
{
    char buffer[10];
    itoa(i, buffer, 10);
    HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    if(m == 1) HAL_UART_Transmit(huart, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

//
// Send text to PC
//
void UART_SEND_TXT(UART_HandleTypeDef *huart, char buffer[], int m)
{
    HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    if(m == 1) HAL_UART_Transmit(huart, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

//
// Start of main program
//
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  HAL_I2C_Init(&hi2c1);
  MX_USART2_UART_Init();

  LPS22HB_Init();

  while (1)
  {
	    UART_SEND_TXT(&huart2, "Temperature = ", 0);		// Send heading
	  	tempC = LPS22HB_GetTemperature();					// GEt temp
	  	UART_SEND_INT(&huart2, tempC, 1);					// Send temp
        HAL_Delay(1000);									// Wait 1 sec
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
