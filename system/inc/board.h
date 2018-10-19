#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "timer.h"
#include "uart.h"
/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PA_8

#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13
#define RADIO_NSS                                   PB_12

#define RADIO_DIO_0                                 PC_6
#define RADIO_DIO_1                                 PC_7
#define RADIO_DIO_2                                 PC_8
#define RADIO_DIO_3                                 PC_9
#define RADIO_DIO_4                                 PB_3
#define RADIO_DIO_5                                 PA_15

#define RADIO_ANT_SWITCH_HF                         PA_0
#define RADIO_ANT_SWITCH_LF                         PA_1

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define BOARD_LED																		PB_11
/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * Possible power sources
 */
enum BoardPowerSources
{
    USB_POWER = 0,
    BATTERY_POWER,
};

extern Uart_t Uart1;
void USART1_init(void);
void BoardGetUniqueId( uint8_t *id );
void BoardInitMcu( void );
void BoardDeInitMcu( void );
void BoardDisableIrq( void );
void BoardEnableIrq( void );
uint8_t GetBoardPowerSource( void );
#endif
