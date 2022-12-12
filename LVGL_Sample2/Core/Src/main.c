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
#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"
#include "lvgl/lvgl.h"

#include "lvgl/examples/lv_examples.h"
#include "lvgl/demos/lv_demos.h"
#include "display_mng.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_BUFFER_SIZE         (50u)

#define LED_TASK_TIME             (1000u) /* In milliseconds */
#define LVGL_TASK_TIME            (5u)    /* In milliseconds */
#define DISP_MNG_TASK_TIME        (100u)  /* In milliseconds */
#define TRIG_ADC_CONV_TASK_TIME   (100u)  /* In milliseconds */
#define DEBUG_PRINT_TASK_TIME     (1000u) /* In milliseconds */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t led_state = FALSE;
static uint32_t led_timestamp = 0u;
static uint32_t lvgl_timestamp = 0u;
static uint32_t disp_mng_timestamp = 0u;
static uint32_t trig_adc_conv_timestamp = 0u;
static uint32_t debug_print_timestamp = 0u;

static uint8_t adc_data;
static uint8_t adc_busy = FALSE;
// ADC Triggering Task Time is 100ms, this means that to get 1 second counts
// we need to store 10 samples which are 100ms apart from each other
static uint8_t temp_sensor[10u] = { 0x00 };
static uint8_t temp_sensor_idx = 0u;
// the above samples are averaged and stored in the below array
static uint8_t temp_sensor_1sec[260] = { 0 };   // 320-60
static uint16_t temp_sensor_1sec_idx = 0u;

uint16_t dbg_size = 0u;
char dbg_buffer[DEBUG_BUFFER_SIZE] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  /* USER CODE BEGIN 2 */
  lv_init();
  tft_init();
  touchpad_init();

  // lv_example_get_started_1();
  // lv_demo_widgets();
  led_state = FALSE;

  lvgl_timestamp = HAL_GetTick();
  led_timestamp = HAL_GetTick();
  trig_adc_conv_timestamp = HAL_GetTick();
  debug_print_timestamp = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Task for Triggering ADC Conversion Start */
    if( HAL_GetTick() - trig_adc_conv_timestamp > TRIG_ADC_CONV_TASK_TIME )
    {
      // TODO: XS
      trig_adc_conv_timestamp = HAL_GetTick();
    }

    /* Task for Printing debug information */
    if( HAL_GetTick() - debug_print_timestamp > DEBUG_PRINT_TASK_TIME )
    {
      debug_print_timestamp = HAL_GetTick();
      // TODO: XS for future
    }

    /* Task Display Manager */
    if( HAL_GetTick() - disp_mng_timestamp > DISP_MNG_TASK_TIME )
    {
      disp_mng_timestamp = HAL_GetTick();
      Display_Mng();
    }

    /* Task for LVGL */
    if( HAL_GetTick() - lvgl_timestamp > LVGL_TASK_TIME )
    {
      lvgl_timestamp = HAL_GetTick();
      lv_timer_handler();
    }

    /* Task for Leds */
    if( HAL_GetTick() - led_timestamp > LED_TASK_TIME )
    {
      led_timestamp = HAL_GetTick();
      if( led_state )
      {
        led_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin(LD_USER1_GPIO_Port, LD_USER1_Pin, GPIO_PIN_RESET );
        HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin(LD_USER1_GPIO_Port, LD_USER1_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin(LD_USER2_GPIO_Port, LD_USER2_Pin, GPIO_PIN_SET );
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
