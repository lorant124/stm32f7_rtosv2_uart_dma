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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ringbuff.h"
#include "minmea.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern osMessageQueueId_t usart_rx_dma_queue_idHandle;

static ubx_prt[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x23, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x89};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_process_data2(const void* data, size_t len);
void usart_send_string(const char* str);
void usart_rx_dma_thread(void* arg);
void DMA1_Stream5_IRQHandlerX(void);
void USART2_IRQHandlerX(void);
uint8_t usart_start_tx_dma_transfer(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
static uint8_t usart_rx_dma_buffer[500];

 ringbuff_t usart_rx_dma_ringbuff;
 uint8_t usart_rx_dma_ringbuff_data[500];
 ringbuff_t usart_tx_dma_ringbuff;
 uint8_t usart_tx_dma_ringbuff_data[500];
 size_t usart_tx_dma_current_len = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const osThreadAttr_t myTask03_attributes = {
  .name = "usart_rx_dma_thread",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 1000
};
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
  ringbuff_init(&usart_tx_dma_ringbuff, usart_tx_dma_ringbuff_data, sizeof(usart_tx_dma_ringbuff_data));
  ringbuff_init(&usart_rx_dma_ringbuff, usart_rx_dma_ringbuff_data, sizeof(usart_rx_dma_ringbuff_data));

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&USART2->RDR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)usart_rx_dma_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, ARRAY_LEN(usart_rx_dma_buffer));

  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_EnableIT_IDLE(USART2);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)&UART4->TDR);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
  LL_USART_EnableDMAReq_TX(UART4);

  MX_FREERTOS_Init();
  osThreadNew(usart_rx_dma_thread, NULL, &myTask03_attributes);
  //gnss_set(USART2 , 28, ubx_prt);

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();
 
  /* Call init function for freertos objects (in freertos.c) */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void DMA1_Stream5_IRQHandlerX(void) {
    void* d = (void *)1;

    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_HT5(DMA1)) {
        LL_DMA_ClearFlag_HT5(DMA1);             /* Clear half-transfer complete flag */
        osMessageQueuePut(usart_rx_dma_queue_idHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_5) && LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);             /* Clear transfer complete flag */
        osMessageQueuePut(usart_rx_dma_queue_idHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }

    /* Implement other events when needed */
}

void DMA1_Stream4_IRQHandlerX(void) {
    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_STREAM_4) && LL_DMA_IsActiveFlag_TC4(DMA1)) {
        LL_DMA_ClearFlag_TC4(DMA1);             /* Clear transfer complete flag */
        ringbuff_skip(&usart_tx_dma_ringbuff, usart_tx_dma_current_len);/* Skip buffer, it has been successfully sent out */
        usart_tx_dma_current_len = 0;           /* Reset data length */
        usart_start_tx_dma_transfer();          /* Start new transfer */
    }

    /* Implement other events when needed */
}

void USART2_IRQHandlerX(void) {
    void* d = (void *)1;

    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(USART2) && LL_USART_IsActiveFlag_IDLE(USART2)) {
        LL_USART_ClearFlag_IDLE(USART2);        /* Clear IDLE line flag */
        osMessageQueuePut(usart_rx_dma_queue_idHandle, &d, 0, 0); /* Write data to queue. Do not use wait function! */
    }
    /* Implement other events when needed */
}

uint8_t
usart_start_tx_dma_transfer(void) {
    uint32_t old_primask;
    uint8_t started = 0;

    /* Check if DMA is active */
    /* Must be set to 0 */
    old_primask = __get_PRIMASK();
    __disable_irq();

    /* Check if transfer is not active */
    if (usart_tx_dma_current_len == 0) {
        /* Check if something to send  */
        usart_tx_dma_current_len = ringbuff_get_linear_block_read_length(&usart_tx_dma_ringbuff);
        if (usart_tx_dma_current_len > 0) {
            /* Disable channel if enabled */
        	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);

            /* Clear all flags */
            LL_DMA_ClearFlag_TC4(DMA1);
            LL_DMA_ClearFlag_HT4(DMA1);
           // LL_DMA_ClearFlag_GI3(DMA1);
            LL_DMA_ClearFlag_TE4(DMA1);
            LL_DMA_ClearFlag_DME4(DMA1);
            LL_DMA_ClearFlag_FE4(DMA1);
            /* Start DMA transfer */
            LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, usart_tx_dma_current_len);
            LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)ringbuff_get_linear_block_read_address(&usart_tx_dma_ringbuff));

            /* Start new transfer */
            LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
            started = 1;
        }
    }

    __set_PRIMASK(old_primask);

    return started;
}
void usart_send_string(const char* str) {
    //usart_process_data(str, strlen(str));
    ringbuff_write(&usart_rx_dma_ringbuff, str, strlen(str));   /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();              /* Then try to start transfer */
}

void usart_process_data(const void* data, size_t len) {


    /*
     * This function is called on DMA TC and HT events, aswell as on UART IDLE (if enabled) line event.
     *
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
   /*const uint8_t* d = data;
   for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(UART4, *d);
        while (!LL_USART_IsActiveFlag_TXE(UART4)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(UART4)) {}*/

	ringbuff_write(&usart_rx_dma_ringbuff, data, len);
}

void usart_process_data2(const void* data, size_t len) {


    /*
     * This function is called on DMA TC and HT events, aswell as on UART IDLE (if enabled) line event.
     *
     * For the sake of this example, function does a loop-back data over UART in polling mode.
     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
     */
   const uint8_t* d = data;
   for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(UART4, *d);
        while (!LL_USART_IsActiveFlag_TXE(UART4)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(UART4)) {}


}

int16_t find_str_end(uint8_t *str, uint16_t old_pos, uint16_t pos)
{
	uint16_t i;
	int16_t ret = -1;
	for(i = old_pos; i < pos; i++)
	{
		if(str[i] == '*' )
		{
			ret = i + 5;
			return ret;
		}
		else
			ret = -1;
	}

	return ret;

}


void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;


    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);
    if (pos != old_pos) {
        if (pos > old_pos) {
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
        } else {

            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);

            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }
    old_pos = pos;

    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
}

void usart_rx_dma_thread(void* arg) {
    void* d;

    /* Notify user to start sending data */
    usart_send_string("USART DMA example: DMA HT & TC + USART IDLE LINE IRQ + RTOS processing\r\n");
    usart_send_string("Start sending data to STM32\r\n");

    while (1) {
        /* Block thread and wait for event to process USART data */
        osMessageQueueGet(usart_rx_dma_queue_idHandle, &d, NULL, osWaitForever);

        /* Simply call processing function */
        usart_rx_check();

        (void)d;
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
