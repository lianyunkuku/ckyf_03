/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ps2.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Encoder EncodeA,EncodeB,EncodeC,EncodeD;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern osMessageQueueId_t MotorCSpeedQueue04Handle;
extern osMessageQueueId_t MPUyawDataQueue01Handle;
extern osMessageQueueId_t MotorASpeedQueue02Handle;
extern osMessageQueueId_t MotorBSpeedQueue03Handle;
extern osMessageQueueId_t MotorDSpeedQueue05Handle;
extern osMessageQueueId_t MotorAPosiQueue07Handle;
extern osMessageQueueId_t MotorBPosiQueue07Handle;
extern osMessageQueueId_t MotorCPosiQueue08Handle;
extern osMessageQueueId_t MotorDPosiQueue09Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	PS2_SetInit();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
    if(GPIO_Pin==E1A_Pin||GPIO_Pin==E1B_Pin){
      EncodeProcess(E1A_GPIO_Port,E1A_Pin,E1B_Pin,&EncodeA);
    }
    if(GPIO_Pin==E2A_Pin||GPIO_Pin==E2B_Pin){
      EncodeProcess(E2A_GPIO_Port,E2A_Pin,E2B_Pin,&EncodeB);
    }
    if(GPIO_Pin==E3A_Pin||GPIO_Pin==E3B_Pin){
      EncodeProcess(E3A_GPIO_Port,E3A_Pin,E3B_Pin,&EncodeC);
    }
    if(GPIO_Pin==E4A_Pin||GPIO_Pin==E4B_Pin){
      EncodeProcess(E4A_GPIO_Port,E4A_Pin,E4B_Pin,&EncodeD);
    }
  }
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim->Instance == TIM3){
    float MotorSpeedA=0,MotorSpeedB=0,MotorSpeedC=0,MotorSpeedD=0;
    float MotorAPosi=0,MotorBPosi=0,MotorCPosi=0,MotorDPosi=0;
    MotorAPosi=EncodeA.cnt,MotorBPosi=EncodeB.cnt,MotorCPosi=EncodeC.cnt,MotorDPosi=EncodeD.cnt;
    MotorSpeedA=(float)MotorAPosi-EncodeA.last_cnt;
    EncodeA.last_cnt=MotorAPosi;
    MotorSpeedB=(float)MotorBPosi-EncodeB.last_cnt;
    EncodeB.last_cnt=MotorBPosi;
    MotorSpeedC=-(float)(MotorCPosi-EncodeC.last_cnt);
    EncodeC.last_cnt=MotorCPosi;
    MotorSpeedD=-(float)(MotorDPosi-EncodeD.last_cnt);
    EncodeD.last_cnt=MotorDPosi;
    osMessageQueuePut(MotorASpeedQueue02Handle,&MotorSpeedA,0,0);
    osMessageQueuePut(MotorBSpeedQueue03Handle,&MotorSpeedB,0,0);
    osMessageQueuePut(MotorCSpeedQueue04Handle,&MotorSpeedC,0,0);
    osMessageQueuePut(MotorDSpeedQueue05Handle,&MotorSpeedD,0,0);
    osMessageQueuePut(MotorAPosiQueue07Handle,&MotorAPosi,0,0);
    osMessageQueuePut(MotorBPosiQueue07Handle,&MotorBPosi,0,0);
    osMessageQueuePut(MotorCPosiQueue08Handle,&MotorCPosi,0,0);
    osMessageQueuePut(MotorDPosiQueue09Handle,&MotorDPosi,0,0);
  }
  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
