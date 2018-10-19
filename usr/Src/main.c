
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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "cmsis_os.h"
#include "rtc.h"
#include "usb_device.h"
#include "gpio.h"
#include "delay.h"
#include "board.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "lora_test.h"
#include "usbd_cdc_if.h"

#include "uart.h"
void MX_FREERTOS_Init(void);
uint8_t str[128];
uint8_t temp;
int main(void)
{
	uint8_t timeout;
	BoardInitMcu();
	USART1_init();
	MX_GPIO_Init();
//  MX_RTC_Init();
//	SX1276IoInit( );
	SX1276Reset();
	GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
	SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
	GpioInit( &LED, BOARD_LED, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_PULL_UP, SET );
	sprintf((char*)str,"LoRa SX1278\r\n");
	uart1_send(str,(uint16_t)strlen((char*)str));
//  MX_FREERTOS_Init();
	SX1276Write( REG_LR_OPMODE, 0x88);
	temp = SX1276Read(REG_LR_OPMODE);
	sprintf((char*)str,"REG_LR_IRQFLAGS = 0x%02x \r\n",temp);
	uart1_send(str,(uint16_t)strlen((char*)str));
	SX1276Write(REG_LR_OCP,0x0B);
	SX1276Write(REG_LR_LNA,0x23);//0x20(默认值Default LNA current)
	SX1276Write(REG_LR_MODEMCONFIG1,0x72);
	SX1276Write(REG_LR_MODEMCONFIG2,0x73);
	SX1276Write(REG_LR_PREAMBLEMSB,0x00);
	SX1276Write(REG_LR_PREAMBLELSB,12);
	SX1276Write(REG_LR_DIOMAPPING2,0x01);
	
	DelayMs(10);
//  osKernelStart();
	timeout = 20;
  while (1)
  {
			
		
		SX1276Write( REG_LR_OPMODE, 0x89);
		DelayMs(1);
		temp = SX1276Read(REG_LR_OPMODE);
		sprintf((char*)str,"RFLR_OPMODE_STANDBY = 0x%02x \r\n",temp);
		uart1_send(str,(uint16_t)strlen((char*)str));
		SX1276Write(REG_LR_PADAC,0x87);
		SX1276Write(REG_LR_HOPPERIOD,0x00);
		SX1276Write(REG_LR_DIOMAPPING1,0x41);
		SX1276Write(REG_LR_IRQFLAGSMASK,0xF7);
		SX1276Write(REG_LR_IRQFLAGS,0xFF);
		sprintf((char*)str,"LoRa SX1278\r\n");
		SX1276Write(REG_LR_PAYLOADLENGTH,strlen((char*)str));
		SX1276Write(REG_LR_FIFOADDRPTR,SX1276Read(REG_LR_FIFOTXBASEADDR));
		SX1276WriteFifo(str,(uint8_t)strlen((char*)str));
		SX1276Write(REG_LR_OPMODE,0x8B);
		DelayMs(1);
		temp = SX1276Read(REG_LR_OPMODE);
		sprintf((char*)str,"RFLR_OPMODE_TRANSMITTER = 0x%02x \r\n",temp);
		uart1_send(str,(uint16_t)strlen((char*)str));
		while(timeout>0)
		{
			DelayMs(100);
			timeout--;
			if(GpioRead(&SX1276.DIO0) == true)
			{
				sprintf((char*)str,"Tx Done! \r\n");
				uart1_send(str,(uint16_t)strlen((char*)str));
				SX1276Write(REG_LR_IRQFLAGS,0xFF);
				SX1276Write( REG_LR_OPMODE, 0x88 );
				Lightning();
				break;
			}
		}
		if(timeout == 0)
		{	
			SX1276Write( REG_LR_OPMODE, 0x89);
			sprintf((char*)str,"Tx timeout! \r\n");
			uart1_send(str,(uint16_t)strlen((char*)str));
			SX1276Write(REG_LR_OCP,0x0B);
			SX1276Write(REG_LR_LNA,0x23);//0x20(默认值Default LNA current)
			SX1276Write(REG_LR_MODEMCONFIG1,0x72);
			SX1276Write(REG_LR_MODEMCONFIG2,0x73);
			SX1276Write(REG_LR_PREAMBLEMSB,0x00);
			SX1276Write(REG_LR_PREAMBLELSB,12);
			SX1276Write(REG_LR_DIOMAPPING2,0x01);
			SX1276Write( REG_LR_OPMODE, 0x88 );
			Lightning2();
		}	
		temp = SX1276Read(REG_LR_OPMODE);
		sprintf((char*)str,"RFLR_OPMODE_SLEEP = 0x%02x \r\n",temp);
		uart1_send(str,(uint16_t)strlen((char*)str));
		sprintf((char*)str,"================================\r\n");
		uart1_send(str,(uint16_t)strlen((char*)str));
		timeout = 20;
		DelayMs(1000);
		
		
  }

}


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
