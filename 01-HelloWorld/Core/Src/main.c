/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_1_TASK_TIME       2000u   /* In milliseconds */
#define LED_2_TASK_TIME       1000u   /* In milliseconds */
#define DBG_UART_TASK_TIME    1000u   /* In milliseconds */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t led_1_state = FALSE;
uint8_t led_2_state = FALSE;
uint8_t led_3_state = FALSE;
uint8_t led_3_toggle = FALSE;

uint32_t led_1_timestamp = 0u;
uint32_t led_2_timestamp = 0u;
uint32_t dbg_uart_timestamp = 0u;

uint8_t rx_data = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  led_1_timestamp = HAL_GetTick();
  led_2_timestamp = HAL_GetTick();
  dbg_uart_timestamp = HAL_GetTick();

  HAL_UART_Receive_IT(&huart1, &rx_data, 1u);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*----Method-1: Turning On and Off Individual Led's
    HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
    HAL_Delay(1000);
    HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
    HAL_Delay(1000);
    Method-1 Ends---*/
    /*----Method-2: Turning On and Off Individual Led's Using Toggle Function
    HAL_GPIO_TogglePin( LD1_GPIO_Port, LD1_Pin );
    HAL_GPIO_TogglePin( LD2_GPIO_Port, LD2_Pin );
    HAL_Delay(1000);
    Method-1 Ends---*/
    /*---Method-3: With the help of SysTick Timer---*/
    /* Task for Led 1 */
    if( HAL_GetTick() - led_1_timestamp > LED_1_TASK_TIME )
    {
      led_1_timestamp = HAL_GetTick();
      if( led_1_state )
      {
        led_1_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_1_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET );
      }
    }
    /* Task for Led 2 */
    if( HAL_GetTick() - led_2_timestamp > LED_2_TASK_TIME )
    {
      led_2_timestamp = HAL_GetTick();
      if( led_2_state )
      {
        led_2_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_2_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET );
      }
    }
    /*--External Interrupt Example, led_3_toggle variable is set inside the EXT Interrupt  --*/
    if( led_3_toggle == TRUE )
    {
      led_3_toggle = FALSE;
      /*--Below Function can be used directly, but to display on CubeMonitor, I am using this method--*/
      // HAL_GPIO_TogglePin( LD3_GPIO_Port, LD3_Pin );
      if( led_3_state )
      {
        led_3_state = FALSE;
        HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_3_state = TRUE;
        HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
      }
    }
    /*--UART1 is connected to VCOM of PC using ST-Link--*/
    if( HAL_GetTick() - dbg_uart_timestamp > DBG_UART_TASK_TIME )
    {
      uint8_t text[] = "Hello World from STM32F7\r\n";
      dbg_uart_timestamp = HAL_GetTick();
      // NOTE: length is sizeof(text)-1, else NULL character will also be transmitted, which is not needed
      // if( HAL_UART_Transmit(&huart1, text, (sizeof(text)-1u), 1000u) != HAL_OK )
      if( HAL_UART_Transmit_IT(&huart1, text, (sizeof(text)-1u) ) != HAL_OK )
      {
        /*--All HAL functions returns HAL_OK, if not call Erro Handler function for analysis of fault--*/
        Error_Handler();
      }
    }
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  if( GPIO_Pin == BLUE_BUTTON_Pin )
  {
    /*--Set this flag to toggle the LD3 on the board, whenever button is pressed--*/
    led_3_toggle = TRUE;
    /*--NOTE: there is no need to clear the flags as they are already cleared in the previous functions--*/
  }
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if (huart->Instance == USART1)
  {
    HAL_UART_Transmit_IT(huart, &rx_data, 1u );
    HAL_UART_Receive_IT(huart, &rx_data, 1u );
  }
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
