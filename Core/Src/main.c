/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void ssd1306_Startup() {
//	uint32_t delta;
//
//	  for(delta = 0; delta < 5; delta ++) {
//	    ssd1306_DrawCircle(20* delta+30, 15, 10, White);
//	  }
	ssd1306_WriteString("Welcome", Font_11x18, Black);
	ssd1306_SetCursor(2, 18*2);
	ssd1306_WriteString("Configuring...", Font_11x18, Black);
	  ssd1306_UpdateScreen();
	  return;
}

void Program() {
	ssd1306_Fill(White);

	    uint32_t start = HAL_GetTick();
	    uint32_t end = start;
	    int fps = 0;
	    char message[200][15] = {"468.04ppm         ",
	    		"378.88ppm         ",
	    		"456.62ppm         ",
	    		"459.69ppm         ",
	    		"482.33ppm         ",
	    		"339.51ppm         ",
	    		"367.04ppm         ",
	    		"453.65ppm         ",
	    		"355.55ppm         ",
	    		"410.79ppm         ",
	    		"395.48ppm         ",
	    		"425.77ppm         ",
	    		"372.96ppm         ",
	    		"402.68ppm         ",
	    		"490.45ppm         ",
	    		"483.24ppm         ",
	    		"427.14ppm         ",
	    		"443.46ppm         ",
	    		"328.32ppm         ",
	    		"421.39ppm         ",
	    		"303.26ppm         ",
	    		"348.58ppm         ",
	    		"327.45ppm         ",
	    		"460.84ppm         ",
	    		"331.34ppm         ",
	    		"380.19ppm         ",
	    		"325.96ppm         ",
	    		"321.76ppm         ",
	    		"499.78ppm         ",
	    		"343.65ppm         ",
	    		"402.59ppm         ",
	    		"467.82ppm         ",
	    		"422.53ppm         ",
	    		"359.21ppm         ",
	    		"427.51ppm         ",
	    		"404.86ppm         ",
	    		"398.72ppm         ",
	    		"494.55ppm         ",
	    		"358.50ppm         ",
	    		"454.27ppm         ",
	    		"405.35ppm         ",
	    		"453.98ppm         ",
	    		"380.05ppm         ",
	    		"478.31ppm         ",
	    		"356.66ppm         ",
	    		"370.49ppm         ",
	    		"461.54ppm         ",
	    		"483.81ppm         ",
	    		"313.95ppm         ",
	    		"489.87ppm         ",
	    		"405.20ppm         ",
	    		"317.21ppm         ",
	    		"338.44ppm         ",
	    		"432.65ppm         ",
	    		"478.05ppm         ",
	    		"369.78ppm         ",
	    		"312.83ppm         ",
	    		"304.00ppm         ",
	    		"391.54ppm         ",
	    		"312.62ppm         ",
	    		"347.66ppm         ",
	    		"494.13ppm         ",
	    		"480.44ppm         ",
	    		"470.18ppm         ",
	    		"353.33ppm         ",
	    		"407.95ppm         ",
	    		"375.04ppm         ",
	    		"452.05ppm         ",
	    		"402.51ppm         ",
	    		"433.54ppm         ",
	    		"406.32ppm         ",
	    		"307.86ppm         ",
	    		"387.53ppm         ",
	    		"486.37ppm         ",
	    		"486.16ppm         ",
	    		"444.19ppm         ",
	    		"356.86ppm         ",
	    		"447.71ppm         ",
	    		"428.00ppm         ",
	    		"370.81ppm         ",
	    		"437.57ppm         ",
	    		"333.19ppm         ",
	    		"388.02ppm         ",
	    		"476.02ppm         ",
	    		"465.84ppm         ",
	    		"366.07ppm         ",
	    		"345.79ppm         ",
	    		"478.67ppm         ",
	    		"370.07ppm         ",
	    		"437.33ppm         ",
	    		"491.29ppm         ",
	    		"417.73ppm         ",
	    		"431.46ppm         ",
	    		"471.74ppm         ",
	    		"387.91ppm         ",
	    		"484.79ppm         ",
	    		"379.69ppm         ",
	    		"462.95ppm         ",
	    		"436.84ppm         ",
	    		"482.19ppm         "};

	    ssd1306_SetCursor(2,0);
	    ssd1306_WriteString("Air Quality", Font_11x18, Black);
	    ssd1306_SetCursor(2, 18*2);

	    int count = 0;

	    do {
//	        char ch = message[0];
//	        memmove(message, message+1, sizeof(message)-2);
//	        message[sizeof(message)-2] = ch;

	    	for (int i = 0; i < 10000; i++) {
	    		ssd1306_SetCursor(2, 18);
	    		ssd1306_WriteString(message[rand() % 100], Font_11x18, Black);
	    		HAL_Delay(3000);
	    		ssd1306_UpdateScreen();
	    	}
//	        if (count % 2) {
//	        	ssd1306_SetCursor(2, 18);
//	        	ssd1306_WriteString(message, Font_11x18, Black);
//	        	HAL_Delay(1000);
//	        	ssd1306_UpdateScreen();
//	        } else {
//	        	ssd1306_SetCursor(2, 18);
//	        	ssd1306_WriteString("101", Font_11x18, Black);
//	        	HAL_Delay(1000);
//	        	ssd1306_UpdateScreen();
//	        }

	        fps++;
	        count++;
	        end = HAL_GetTick();
	    } while((end - start) < 50000000);

	    HAL_Delay(50000000);

	    char buff[64];
	    fps = (float)fps / ((end - start) / 1000.0);
	    snprintf(buff, sizeof(buff), "~%d FPS", fps);

	    ssd1306_Fill(White);
	    ssd1306_SetCursor(2, 2);
	    ssd1306_WriteString(buff, Font_11x18, Black);
	    ssd1306_UpdateScreen();
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  while (1)
  {
    /* USER CODE END WHILE */
//	  ssd1306_TestAll();
	  if (count == 0) {
	  			  ssd1306_Startup();
	  			  HAL_Delay(10000);
	  			  count++;
	  	      }
	  for (int i = 0; i < 10000; i++) {
		  Program();
	  }
	  }
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
