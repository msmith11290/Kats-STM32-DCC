/*
 * serialCom.c
 * 		This function interfaces with the Serial Communications channels used in our project such as UARTs and USBs. I2C has its
 * 	own	source file(i2c.c).
 *
 * 	We have two USB channels.
 * 	The first is the only on board USB that the customer will use, J-5 or .
 *
 * 	The second being the USB channel integrated into the debug USB port only available to Me on the STM Nucleo-64 board (to my knowledge,
 * 	so far). This is huart2 I don't think it will be available on the J5 SWD connector. Device Manager sees's this as COM4. On the device
 * 	this is huart2
 *
 * We have two USART channels available.
 *
 * USART 2 (huart2 Pins PA2 & PA3) is the channel integrated into the debug channel on the STM Nucleo-64 board. This can only be used during
 * code developement ( I think ).

 * USART 2 was inadvertently used for the touch screen before I found out it was used by the debugger. Right now I don't know if
 * this is going to work for both the debugger and the touchscreen simultanously.
 *
 * The second channel is USART 3 which didn't get wired on the PCB. (huart3 Pins PA2 & PA3) which is available on J-7. This was
 * intended to drive the touchscreen but is usable by anything else. I would like to include a TTL to USB chip (FTDI FT232RL) on
 * the next design iteration for an alternate USB port.
 *
 *  Created on: Sep 1, 2022
 *      Author: msmit
 */
#include <stdio.h>
#include "usbd_cdc_if.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t RxBuff[50], TxBuf[50];
bool dataAvailable = false;
/**********************************************************************************************************************************
 * debugPrint( char *msg, int size )
 * Prints a message "msg" of size characters. The print is accomplished using huart2 which is integrated into the debugger. After
 * hooking up the debugger, no other wires or cables are required.
 *********************************************************************************************************************************/
void debugPrint( char *msg, int size )
{
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, size, size);
}

/**********************************************************************************************************************************
 * uart2_test( void) : Prolific USB-to-Serial Comm Port(COM 8) : Pins PA2 & PA3 (32F103)
 * Is only intended as a test and not a very good one at that because if it doesn't work, no message is displayed so the user has
 * no way of knowing it doesn't work.
 *********************************************************************************************************************************/
void uart2_test(void)
{
	int uart_buf_len;
	if (HAL_UART_Init(&huart2) == HAL_OK) //USART_2 Pins PA2 & PA3 CN10 Pins p35 & 37
	{
		uart_buf_len = sprintf( (char *)&TxBuf, "Serial over Touchscreen UART 2 Test PASSED\r\n");
		HAL_UART_Transmit(&huart2, TxBuf, uart_buf_len, 100);
	}
}


/**********************************************************************************************************************************
 * uart3_test( void) : 32F103 Pins PB10 & PB11 : Not connected up on our board
 * Is only intended as a test and not a very good one at that because it's not wired up on the PCB. It can only be tested on the
 *
 *********************************************************************************************************************************/
void uart3_test( void)
{
	int buf_len;
	if (HAL_UART_Init(&huart3) == HAL_OK)
	{
	  buf_len = sprintf((char *)TxBuf, "Non-Existant UART 3 Test\r\n");
	  HAL_UART_Transmit(&huart3, TxBuf, buf_len, 100);
	}   //End of superceded by using virtual port 2
}
/**********************************************************************************************************************************
 * TransmitData : Send data to KATS PC
 * For this set of PCB's we use USART 2
 *********************************************************************************************************************************/
void TransmitData( uint8_t buff, int size )
{
	HAL_UART_Transmit(&huart2, TxBuf, size, 100);
}
