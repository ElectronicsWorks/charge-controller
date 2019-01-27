/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
#define LOG_PERIOD_MS				1000

#define	CURRENT_THRESHOLD 	745 	// 60 mv/A * 10 A / 3300 mV * 4095
#define BATTERY_1_THRESHOLD	845		// 3.6 V / 1250 Ohm * 250 Ohm / 3.3 V * 4095
#define BATTERY_2_THRESHOLD	1160	// 3.6 V / 1350 Ohm * 350 Ohm / 3.3 V * 4095
#define BATTERY_3_THRESHOLD	1760	// 3.6 V / 1650 Ohm * 650 Ohm / 3.3 V * 4095
#define BATTERY_4_THRESHOLD	3515	// 3.6 V / 4700 Ohm * 3700 Ohm / 3.3 V * 4095

#define WINDOW_SIZE					100

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"

//raw adc data from main
extern volatile uint16_t adc_array[];

//uart rx buf from main
extern uint8_t uart_rx_buf[];

//filtered adc data
uint16_t data_array[CHANNELS_NUMBER];

//filtration tools
uint32_t sum_array[CHANNELS_NUMBER];
uint16_t buffer_array[CHANNELS_NUMBER][WINDOW_SIZE];
uint16_t index_array[CHANNELS_NUMBER];
bool state_array[CHANNELS_NUMBER];

//thresholds
uint16_t c_threshold = CURRENT_THRESHOLD;
uint16_t v1_threshold = BATTERY_1_THRESHOLD;
uint16_t v2_threshold = BATTERY_2_THRESHOLD;
uint16_t v3_threshold = BATTERY_3_THRESHOLD;
uint16_t v4_threshold = BATTERY_4_THRESHOLD;

//parameters for logging
double parameters_h[CHANNELS_NUMBER];
uint8_t relay_state = 0;
uint16_t coefficients[CHANNELS_NUMBER] = {75, 235, 322, 489, 975};
//log msg
char msg[150];
uint16_t log_period = LOG_PERIOD_MS;
uint16_t delay_ms = 0;

//filtering func
uint16_t moving_average(uint16_t in, uint32_t *sum, uint16_t *buffer_array, uint16_t *index, uint16_t max_index, bool *buffer_state);
void ReInit_Timer(uint16_t ms);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if (++delay_ms >= log_period)
	{
		//reset counter
		delay_ms = 0;
		//perform data to human-friendly view
		for (int i = 0; i < CHANNELS_NUMBER; i++) parameters_h[i] = (double) data_array[i] / coefficients[i];
		//check if relay is on or off
		if (RELAY_GPIO_Port->ODR & (1 << 12)) relay_state = 1;
		else relay_state = 0;
		//perform output string
		sprintf(msg, "Current: %f A\r\nB1 Voltage: %f V\r\nB2 Voltage: %f V\r\nB3 Voltage: %f V\r\nB4 Voltage: %f V\r\nRelay State: %d\r\n\r\n", parameters_h[0], 
		parameters_h[1],
		parameters_h[2], 
		parameters_h[3], 
		parameters_h[4], 
		relay_state);
		//start transmission
		HAL_UART_Transmit_IT(&huart3, (uint8_t *) msg, strlen(msg));
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) 
	&& (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET))
	{
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		uint16_t rx_bytes_number = huart3.RxXferSize - huart3.RxXferCount;
		HAL_UART_Abort_IT(&huart3);
    huart3.RxXferCount = 0;
    if((huart3.gState | huart3.RxState) == HAL_UART_STATE_BUSY_TX_RX)
    {
      huart3.gState = HAL_UART_STATE_BUSY_TX;
    }
    else
    {
      huart3.gState = HAL_UART_STATE_READY;
    }
    huart3.RxState = HAL_UART_STATE_READY;
		char *buf_char = (char *) uart_rx_buf;
		static char err_msg[] = "\r\nERROR!\r\n";
		static char ok_msg[] = "\r\nOK!\r\n";
		
		//set current threshold message received
		if (strstr(buf_char, "SC ") == buf_char)
		{
			c_threshold = atof(buf_char + 3) * 60 / 3300 * 4095;
		}
		//set battery 1 voltage threshold message received
		else if (strstr(buf_char, "SV1 ") == buf_char)
		{
			v1_threshold = atof(buf_char + 3) / 1250 * 250 / 3.3 * 4095;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
		}
		//set battery 2 voltage threshold message received
		else if (strstr(buf_char, "SV2 ") == buf_char)
		{
			v2_threshold = atof(buf_char + 3) / 1350 * 350 / 3.3 * 4095;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
		}
		//set battery 3 voltage threshold message received
		else if (strstr(buf_char, "SV3 ") == buf_char)
		{
			v3_threshold = atof(buf_char + 3) / 1650 * 650 / 3.3 * 4095;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
		}
		//set battery 4 voltage threshold message received
		else if (strstr(buf_char, "SV4 ") == buf_char)
		{
			v4_threshold = atof(buf_char + 3) / 4700 * 3700 / 3.3 * 4095;
			HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
		}
		//set timeout message received
		else if (strstr(buf_char, "ST ") == buf_char)
		{
			//if timer is not going
			if (!(TIM6->CR1 & TIM_CR1_CEN))
			{
				ReInit_Timer(atoi(buf_char + 3));
				HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
			}
			else HAL_UART_Transmit_IT(&huart3, (uint8_t *) err_msg, strlen(err_msg));
		}
		//set new period of logging
		else if (strstr(buf_char, "SL ") == buf_char)
		{
			log_period = atoi(buf_char + 3);
			HAL_UART_Transmit_IT(&huart3, (uint8_t *) ok_msg, strlen(ok_msg));
		}
		//set buffer to zero
		memset(uart_rx_buf, 0, rx_bytes_number);
		//start reception again
		HAL_UART_Receive_IT(&huart3, uart_rx_buf, UART_RX_BUF_SIZE);
	}
  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	//stop timer
	HAL_TIM_Base_Stop_IT(&htim6);
	//if current fell down enough after time out, switch on relay again
	if (data_array[0] < CURRENT_THRESHOLD) HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	//perform raw data
	for (int i = 0; i < CHANNELS_NUMBER; i++) data_array[i] = moving_average(adc_array[i], &sum_array[i], &buffer_array[i][0], &index_array[i], WINDOW_SIZE, &state_array[i]);
	//if some parameter is out of range and timer is not going, start timer and switch off relay
	if (((data_array[0] > c_threshold)
		|| (data_array[1] < v1_threshold) 
		|| (data_array[2] < v2_threshold)
		|| (data_array[3] < v3_threshold)
		|| (data_array[4] < v4_threshold)) && (!(TIM6->CR1 & TIM_CR1_CEN)))
	{
		__HAL_TIM_SetCounter(&htim6, 0);
		HAL_TIM_Base_Start_IT(&htim6);
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
	}
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint16_t moving_average(uint16_t in, uint32_t *sum, uint16_t *buffer_array, uint16_t *index, uint16_t max_index, bool *buffer_state)
{
	if (*buffer_state == true)
	{
		*sum -= buffer_array[*index];
	}
	*sum += in;
	buffer_array[*index] = in;
	*index += 1;
	if (*index == max_index)
	{
		*buffer_state = true;
		*index = 0;
	}
	if (buffer_state) return *sum / max_index;
	else return *sum / *index;
}

void ReInit_Timer(uint16_t ms)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 41999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = ms * 2;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
