/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ringbuff.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern	 ringbuff_t usart_rx_dma_ringbuff;
extern	 uint8_t usart_rx_dma_ringbuff_data[128];
extern	 ringbuff_t usart_tx_dma_ringbuff;
extern   uint8_t usart_tx_dma_ringbuff_data[128];
extern	 size_t usart_tx_dma_current_len;
extern void usart_send_string(const char* str);
extern uint8_t usart_start_tx_dma_transfer(void);
extern void usart_process_data2(const void* data, size_t len);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t b, state = 0, local_buff[100] = {0};
uint16_t cnt = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1000
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 300
};
/* Definitions for usart_rx_dma_queue_id */
osMessageQueueId_t usart_rx_dma_queue_idHandle;
const osMessageQueueAttr_t usart_rx_dma_queue_id_attributes = {
  .name = "usart_rx_dma_queue_id"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usart_rx_dma_queue_id */
  usart_rx_dma_queue_idHandle = osMessageQueueNew (10, sizeof(uint8_t), &usart_rx_dma_queue_id_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
uint8_t a = 0;
  /* Infinite loop */
	for(;;)
	{

		  if (ringbuff_read(&usart_rx_dma_ringbuff, &b, 1) == 1)
		  {
			  taskENTER_CRITICAL();

	            ringbuff_write(&usart_tx_dma_ringbuff, &b, 1);   /* Write data to transmit buffer */
	            //ringbuff_read(&usart_tx_dma_ringbuff, &a, 1);
	            //usart_process_data2(&a, 1);
	           usart_start_tx_dma_transfer();
	            taskEXIT_CRITICAL();
			  //usart_process_data2(&b, 1);
					  /*switch (state) {
						  case 0:

							  if (b == '$') {
								  local_buff[cnt++] = b;
								  ++state;
							  }
							  break;

						  case 1:
							  if (cnt >= 98 ) {
								  state = 0;
								  cnt = 0;
								  break;
							  }

							  local_buff[cnt++] = b;

							  if (b == '*')
							  {

								  local_buff[cnt++] = '\n';
								  ++state;

								  break;
							  }
							  break;

						  case 2:
							  //usart_process_data2(&local_buff, cnt);

							 //ringbuff_write(&usart_tx_dma_ringbuff, &local_buff, cnt);
							 //usart_start_tx_dma_transfer();


							  cnt = 0;
							  state = 0;

							  break;


					  }*/
		  }
	 }
	       osDelay(1);
  }
  /* USER CODE END StartDefaultTask */


/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t set_flag = 0;
  /* Infinite loop */
  for(;;)
  {
	if(!set_flag)
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,GPIO_PIN_SET);
		set_flag = 1;
	}
	else
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,GPIO_PIN_RESET);
		set_flag = 0;
	}

    osDelay(200);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
