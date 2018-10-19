#ifndef __UART_H__
#define __UART_H__

#include "fifo.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdlib.h"
/*!
 * UART notification identifier
 */
typedef enum
{
    UART_NOTIFY_TX,
    UART_NOTIFY_RX
}UartNotifyId_t;

/*!
 * UART object type definition
 */
typedef struct
{
    uint8_t UartId;
    bool IsInitialized;
    Gpio_t Tx;
    Gpio_t Rx;
    Fifo_t FifoTx;
    Fifo_t FifoRx;
    /*!
     * IRQ user notification callback prototype.
     */
    void ( *IrqNotify )( UartNotifyId_t id );
}Uart_t;

/*!
 * Operation Mode for the UART
 */
typedef enum
{
    TX_ONLY = 0,
    RX_ONLY,
    RX_TX
}UartMode_t;

/*!
 * UART word length
 */
typedef enum
{
    UART_8_BIT = 0,
    UART_9_BIT
}WordLength_t;

/*!
 * UART stop bits
 */
typedef enum
{
    UART_1_STOP_BIT = 0,
    UART_0_5_STOP_BIT,
    UART_2_STOP_BIT,
    UART_1_5_STOP_BIT
}StopBits_t;

/*!
 * UART parity
 */
typedef enum
{
    NO_PARITY = 0,
    EVEN_PARITY,
    ODD_PARITY
}Parity_t;

/*!
 * UART flow control
 */
typedef enum
{
    NO_FLOW_CTRL = 0,
    RTS_FLOW_CTRL,
    CTS_FLOW_CTRL,
    RTS_CTS_FLOW_CTRL
}FlowCtrl_t;

/*!
 * UART peripheral ID
 */
typedef enum
{
    UART_1,
    UART_2,
    UART_USB_CDC = 255,
}UartId_t;


/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 * \param [IN] tx   UART Tx pin name to be used
 * \param [IN] rx   UART Rx pin name to be used
 */
void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx );

/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj          UART object
 * \param [IN] mode         Mode of operation for the UART
 * \param [IN] baudrate     UART baudrate
 * \param [IN] wordLength   packet length
 * \param [IN] stopBits     stop bits setup
 * \param [IN] parity       packet parity
 * \param [IN] flowCtrl     UART flow control
 */
void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl );

/*!
 * \brief DeInitializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 */
void UartMcuDeInit( Uart_t *obj );

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Character to be sent
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data );

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Characters to be sent
 * \param [IN] size  number of characters to send
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuPutBuffer( Uart_t *obj, uint8_t *data, uint16_t size );

/*!
 * \brief Gets a character from the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Received character
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data );

/*!
 * \brief Gets a character from the UART (blocking mode)
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Received character
 * \param [IN] size  number of characters to be received
 * \retval status    [0: OK, 1: Busy]
 */
uint8_t UartMcuGetBuffer( Uart_t *obj, uint8_t *data, uint16_t size );

void uart1_send(uint8_t *pData, uint16_t Size);
#endif




