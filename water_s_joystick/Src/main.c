
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

#include "tm2_stm32_nrf24l01.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

NRF24L01_config_TypeDef nrf_tx_cfg;

uint8_t nrf_init_result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void int8_to_bin( int8_t x, int8_t *_buf );
void usb_receive_interrupt( uint8_t );
void usb_receive_bytes( uint8_t* Buf, uint32_t *Len );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	  // for received data
	  //uint8_t dataIn[32];
	  uint8_t loop;
	  uint8_t text_to_show[ 100 ];

	  // NRF transmission status
	  TM_NRF24L01_Transmit_Status_t transmissionStatus;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  uint8_t tmp_char;

  nrf_tx_cfg.CE_pin          = NRF_CE_Pin;
  nrf_tx_cfg.CE_port         = NRF_CE_GPIO_Port;
  nrf_tx_cfg.CSN_pin         = NRF_CSN_Pin;
  nrf_tx_cfg.CSN_port        = NRF_CSN_GPIO_Port;
  nrf_tx_cfg.SPI             = &hspi1;
  nrf_tx_cfg.radio_channel   = 15;
  nrf_tx_cfg.baud_rate       = TM_NRF24L01_DataRate_1M;
  nrf_tx_cfg.payload_len     = 32;
  nrf_tx_cfg.crc_len         = 2;
  nrf_tx_cfg.output_power    = TM_NRF24L01_OutputPower_0dBm;
  nrf_tx_cfg.rx_address[ 0 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 1 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 2 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 3 ] = 0xE7;
  nrf_tx_cfg.rx_address[ 4 ] = 0xE7;
  nrf_tx_cfg.tx_address[ 0 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 1 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 2 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 3 ] = 0x7E;
  nrf_tx_cfg.tx_address[ 4 ] = 0x7E;

  nrf_init_result = tm2_NRF24L01_Init( &nrf_tx_cfg );
  HAL_Delay( 2000 );
  tm2_NRF24L01_Clear_Interrupts( &nrf_tx_cfg );
  HAL_Delay( 100 );

  //turn off power alarm LED
  HAL_GPIO_WritePin( LED_power_ALARM_GPIO_Port, LED_power_ALARM_Pin, GPIO_PIN_SET );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay( 25 );

	    if( tm2_NRF24L01_DataReady( &nrf_tx_cfg )) {
	    	HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET );
	    	for( loop = 0; loop < 33; loop++ ) text_to_show[ loop ] = 0;
	    	tm2_NRF24L01_GetData( &nrf_tx_cfg, text_to_show );
	    	CDC_Transmit_FS( text_to_show, strlen( text_to_show ));
	    	HAL_Delay( 10 );
	    }
	    else {
	    	HAL_Delay( 25 );
	    }

	  HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_SET );
	  tmp_char = 0x01;
	  if( ! HAL_GPIO_ReadPin( RIGHT_UP_GPIO_Port, RIGHT_UP_Pin )) {
		  tmp_char |= 0x04;
		  HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET );
	  }
	  if( ! HAL_GPIO_ReadPin( RIGHT_DOWN_GPIO_Port, RIGHT_DOWN_Pin )) {
		  tmp_char |= 0x02;
		  HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET );
	  }
	  if( ! HAL_GPIO_ReadPin( LEFT_UP_GPIO_Port, LEFT_UP_Pin )) {
		  tmp_char |= 0x10;
		  HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET );
	  }
	  if( ! HAL_GPIO_ReadPin( LEFT_DOWN_GPIO_Port, LEFT_DOWN_Pin )) {
		  tmp_char |= 0x08;
		  HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET );
	  }
	  //int8_to_bin( tmp_char, text_to_show );
	  //text_to_show[ 8 ] = 13;
	  //text_to_show[ 9 ] = 10;
	  //if( tmp_char != 0 ) ; //CDC_Transmit_FS( text_to_show, 10 );
	  //HAL_Delay( 1 );
	  tm2_NRF24L01_Transmit( &nrf_tx_cfg, &tmp_char );
	  /* Wait for data to be sent */
	  do {
		/* Get transmission status */
		transmissionStatus = tm2_NRF24L01_GetTransmissionStatus( &nrf_tx_cfg );
	  } while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
	  HAL_Delay( 5 );
	  tm2_NRF24L01_PowerUpRx( &nrf_tx_cfg );

	  /* Check transmit status */
//		if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) {
//			/* Transmit went OK */
//			text_to_show[ 0 ] = 'O';
//			text_to_show[ 1 ] = 'K';
//		} else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) {
//			/* Message was LOST */
//			text_to_show[ 0 ] = 'n';
//			text_to_show[ 1 ] = 'n';
//		} else {
//			/* This should never happen */
//
//		}
//		text_to_show[ 2 ] = ' ';
//		if( tmp_char != 0 ) CDC_Transmit_FS( text_to_show, 3 );
//		HAL_Delay( 1 );

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_power_ALARM_Pin|NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_PC13_Pin */
  GPIO_InitStruct.Pin = RED_LED_PC13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_PC13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_power_ALARM_Pin */
  GPIO_InitStruct.Pin = LED_power_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_power_ALARM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_DOWN_Pin RIGHT_UP_Pin LEFT_DOWN_Pin LEFT_UP_Pin */
  GPIO_InitStruct.Pin = RIGHT_DOWN_Pin|RIGHT_UP_Pin|LEFT_DOWN_Pin|LEFT_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void int8_to_bin( int8_t x, int8_t *_buf ) {
	for( int loop = 0; loop < 8; loop++ ) {
		_buf[ loop ] = x & (0x80 >> loop) ? '1' : '0';
	}
	_buf[ 8 ] = '\n';
	_buf[ 9 ] = '\r';
}

void usb_receive_interrupt( uint8_t received_byte ) {
	uint8_t tmp_char;
	uint8_t text_to_show[ 100 ], config_bytes[ 100 ];

	switch( received_byte ) {
	  case 'x' : int8_to_bin( nrf_init_result, text_to_show );
	             text_to_show[  8 ] = 'x';
	             text_to_show[  9 ] = 13;
	             text_to_show[ 10 ] = 10;
	             CDC_Transmit_FS( text_to_show, 11 );
	             HAL_Delay( 100 );
	             break;
	  case 'z' : tm2_NRF24L01_ReadConfig( &nrf_tx_cfg, config_bytes );
	             for( int i = 0; i < 10; i++ ) { //38
		           int8_to_bin( config_bytes[ i ], text_to_show );
		           CDC_Transmit_FS( text_to_show, 10 );
		           HAL_Delay( 1 );
	             }
	             HAL_Delay( 100 );
	             break;
	  default  :
	             break;
	}
}

void usb_receive_bytes( uint8_t* Buf, uint32_t *Len ) {
	switch (Buf[0]) {
	case 'V':
		HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_RESET);
		break;
	case 'v':
		HAL_GPIO_WritePin( RED_LED_PC13_GPIO_Port, RED_LED_PC13_Pin, GPIO_PIN_SET);
		break;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
