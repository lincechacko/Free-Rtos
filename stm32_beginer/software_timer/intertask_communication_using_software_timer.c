/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "retarget.h"
#include <stdio.h>
#include <stdlib.h>
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
UART_HandleTypeDef huart1;

/* Definitions for Sender1 */
osThreadId_t Sender1Handle;
const osThreadAttr_t Sender1_attributes = {
  .name = "Sender1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sender2 */
osThreadId_t Sender2Handle;
const osThreadAttr_t Sender2_attributes = {
  .name = "Sender2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Receiver */
osThreadId_t ReceiverHandle;
const osThreadAttr_t Receiver_attributes = {
  .name = "Receiver",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue */
osMessageQueueId_t myQueueHandle;
const osMessageQueueAttr_t myQueue_attributes = {
  .name = "myQueue"
};
/* Definitions for Periodic_timer */
osTimerId_t Periodic_timerHandle;
const osTimerAttr_t Periodic_timer_attributes = {
  .name = "Periodic_timer"
};
/* Definitions for OnceTimer */
osTimerId_t OnceTimerHandle;
const osTimerAttr_t OnceTimer_attributes = {
  .name = "OnceTimer"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void Sender1Task(void *argument);
void Sender2Task(void *argument);
void ReceiverTask(void *argument);
void PTCallback(void *argument);
void OTCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	int count;
	int num;
	char *string;
}my_str;
int index1=0;
int index2=0;
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
  RetargetInit(&huart1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Periodic_timer */
  Periodic_timerHandle = osTimerNew(PTCallback, osTimerPeriodic, NULL, &Periodic_timer_attributes);

  /* creation of OnceTimer */
  OnceTimerHandle = osTimerNew(OTCallback, osTimerOnce, NULL, &OnceTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue */
  myQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Sender1 */
  Sender1Handle = osThreadNew(Sender1Task, NULL, &Sender1_attributes);

  /* creation of Sender2 */
  Sender2Handle = osThreadNew(Sender2Task, NULL, &Sender2_attributes);

  /* creation of Receiver */
  ReceiverHandle = osThreadNew(ReceiverTask, NULL, &Receiver_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : user_button_Pin */
  GPIO_InitStruct.Pin = user_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(user_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Sender1Task */
/**
  * @brief  Function implementing the Sender1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Sender1Task */
void Sender1Task(void *argument)
{
  /* USER CODE BEGIN 5 */
	osTimerStart(Periodic_timerHandle, 5000);



  /* Infinite loop */
  for(;;)
  {

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
    osDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Sender2Task */
/**
* @brief Function implementing the Sender2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sender2Task */
void Sender2Task(void *argument)
{
  /* USER CODE BEGIN Sender2Task */

  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	  {
		  my_str *ptr2;
		  ptr2=malloc(sizeof(my_str));
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		  ptr2->count=index1+1;
		  ptr2->num=100+index1;
		  ptr2->string="This is from sender 2 with one time timer\r\n";
		  if(osMessageQueuePut(myQueueHandle, &ptr2, 0, 500)==osOK)
		  {
			  printf("success structure from sender 2 is sent to the queue\r\n");
		  }
		  else
		  {
			  printf("failure structure from sender 2  is not send to the queue\r\n");
		  }
		  osTimerStart(OnceTimerHandle, 5000);

	  }
	index1+=1;
    osDelay(100);
  }
  /* USER CODE END Sender2Task */
}

/* USER CODE BEGIN Header_ReceiverTask */
/**
* @brief Function implementing the Receiver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiverTask */
void ReceiverTask(void *argument)
{
  /* USER CODE BEGIN ReceiverTask */

  /* Infinite loop */
  for(;;)
  {
	  my_str *rec;
	  rec=malloc(sizeof(my_str));

	  if(osMessageQueueGet(myQueueHandle, &rec, NULL, osWaitForever)==osOK)
	  {
		  printf("message is received from the queue\r\n");
		  printf("Received structure is\ncount:%d\nnum:%d\nstring:%s\n",rec->count,rec->num,rec->string);
	  }
	  else
	  {
		  printf("message is not received from the queue\r\n");
	  }

    osDelay(1);
  }
  /* USER CODE END ReceiverTask */
}

/* PTCallback function */
void PTCallback(void *argument)
{
  /* USER CODE BEGIN PTCallback */
	                         my_str *ptr1;
		  		  			  ptr1=malloc(sizeof(my_str));

		  		  			  ptr1->count=index2+2;
		  		  			  ptr1->num=100+index2;
		  		  			  ptr1->string="This is from sender 1 with periodic timer\r\n";
		  		  			  if(osMessageQueuePut(myQueueHandle, &ptr1, 0, 500)==osOK)
		  		  			  {
		  		  				  printf("success structure from sender 1 is sent to the queue\r\n");
		  		  			  }
		  		  			  else
		  		  			  {
		  		  				  printf("failure structure from sender 1  is not send to the queue\r\n");
		  		  			  }
		  		  			  index2+=1;
  printf("periodic timer callback is turning off led\r\n");
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
   osDelay(3000);
  /* USER CODE END PTCallback */
}

/* OTCallback function */
void OTCallback(void *argument)
{
  /* USER CODE BEGIN OTCallback */
	printf("one time timer callback is turning off led\r\n");
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
  /* USER CODE END OTCallback */
}

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
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
