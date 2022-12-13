/*
 * i2c.c
 *
 *  Created on: Aug 7, 2022
 *      Author: msmit * I saw how to add files to a project on a video. Besides the method of adding a file
 *
 * The method to add a "c" file is in Project explorer is right click on "Src", then New then Source File.
 * The method to add a "h" file is in Project explorer is right click on "Inc", then New then Header File.
 *
 * I think the most important thing is the include (which I couldn't figure out on my own)
 * #include "stm32f1xx_hal.h" This allowed the HAL_Delay() to compile and work.
 *
 */
#include "stm32f1xx_hal.h"		// Necessary to make calls to HAL calls
//#include "usb_device.h"
#include "i2c.h"
#include "string.h"				// strlen
#include <stdio.h>				// sprintf

extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef ret;
uint8_t outBuf[2], inBuf[2];		// i2c transmits 8 bits at a time, 18 I/O's require two 8 bit bytes
uint8_t msgBuf[50];					// Message Buffer for assembling UART messages
extern UART_HandleTypeDef huart2;

/******************************************************************************************************
 *  The bits default to High after power on so, they are all inputs. No need for modification
 *****************************************************************************************************/
int IO_Expander_read_P0(uint8_t outputBit)
{				// Message Buffer for assembling UART messages
	int ret = 0;
//	outBuf[0] = outBuf[0] ^ outputBit;		// clear outputBit
    // Read 2 bytes from the I/O expander.
   ret = HAL_I2C_Master_Receive(&hi2c1, PCF8575_ADDR, inBuf, TWO_DATA_BYTES, HAL_MAX_DELAY);
   if ( ret == HAL_BUSY )
   {
	  sprintf((char *)msgBuf, "i2c Expander initialization failure \r\n");
	  //strcat((char *)msgBuf,"\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen((char*)msgBuf), 500);
	  ret = -1;
	}
	else
	{
		if(inBuf[0] & outputBit)
			ret = 1;
		else
			ret = 0;
	}
	return ret;
}
/******************************************************************************************************
 The bits default to High after power on so, they are all inputs. Only outputs need modification
 *****************************************************************************************************/
int IO_Expander_set_output_P0(uint8_t outputBit)
{				// Message Buffer for assembling UART messages
	int ret = 0;
//	outBuf[0] = outBuf[0] ^ outputBit;		// clear outputBit
	ret = HAL_I2C_Master_Transmit(&hi2c1, PCF8575_ADDR, outBuf, TWO_DATA_BYTES, HAL_MAX_DELAY);
	if ( ret == HAL_BUSY )
	{
		  sprintf((char *)msgBuf, "i2c Expander initialization failure \r\n");
		    	//strcat((char *)msgBuf,"\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen((char*)msgBuf), 500);
		  ret = -1;
	}
	return ret;
}
int IO_Expander_init(void)
{
	int ret = 0;
	outBuf[0] = 0xFF;					// Make p0.0 thru P0.7 inputs. Po.0 is an input
	outBuf[1] = 0xFF;					// Make p1.0 thru P1.7 inputs

	ret = HAL_I2C_Master_Transmit(&hi2c1, PCF8575_ADDR, outBuf, TWO_DATA_BYTES, HAL_MAX_DELAY);
	if ( ret == HAL_BUSY )
	{
		  sprintf((char *)msgBuf, "i2c Expander initialization failure");
		    	//strcat((char *)msgBuf,"\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen((char*)msgBuf), 500);
	}
	else
	{
		if ( ret != HAL_OK )
		{
		     strcpy((char*)msgBuf, "i2c Transmit FAILURE\r\n");
		     ret = -1;
		}
		else
		{
				// Read 2 bytes from the I/O expander.
			ret = HAL_I2C_Master_Receive(&hi2c1, PCF8575_ADDR, inBuf, TWO_DATA_BYTES, HAL_MAX_DELAY);
			if ( ret != HAL_OK )
			{
				strcpy((char*)msgBuf, "i2c Receive FAILURE\r\n");
			}
	    	else
			{
				  sprintf((char *)msgBuf, "i2c Expander initialization Successful");
				  ret = 1;
			}
		}
		HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen((char*)msgBuf), 500);
	}
	return ret;
}
/******************************************************************************************************
 A small card I bought from Amazon to test I2C
 *****************************************************************************************************/
int i2c_temperature(void)
{
//	HAL_StatusTypeDef ret;
	int ret = 0;
	uint8_t buf[40];		// i2c transmits 8 bits at a time
	int16_t val;
	int fraction;
//	float temp_c;

	memset( buf, 0, sizeof(buf));
	buf[0] = TEMPERATURE_REG;		// Tell LM75 that we want to read register 0. the temperature register
	//buf[0] = ID_REGISTER;
	// Send the device address (7 bits and set R/W bit to W),
	// Followed by a pointer to the register we want to read which is stored in buf[0].
	// Next is the the amount of data from the buffer (buf) that should be sent to the slave device
	// the last is the amount of time we want to allow to finish this i2c Tx operation
	ret = HAL_I2C_Master_Transmit(&hi2c1, LM75_ADDR, buf, ONE_DATA_BYTE, HAL_MAX_DELAY);
	//	    HAL_I2C_Mem_Write (&hi2c1, LM75_ADDR, 0, 1, buf, 1,50); // https://www.youtube.com/watch?v=cvmQNTVJrzs 23:55/37:50
	    	//HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen((char*)buf), 500);
	if ( ret == HAL_BUSY )
	{
		  sprintf((char *)buf, "i2c TX failure");
		    	//strcat((char *)buf,"\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen((char*)buf), 500);
	}
	else
	{
	      // Read 8 bytes from the temperature register.
	     ret = HAL_I2C_Master_Receive(&hi2c1, LM75_ADDR, buf, TWO_DATA_BYTES, HAL_MAX_DELAY);
	     if ( ret != HAL_OK )
	     {
	        strcpy((char*)buf, "i2c Rx failure\r\n");
	     }
	     else
	     {	    				// shift bits 15 <-> 7 to bits 8 <-> 0
	    	 val = buf[0] & 0xFE;			// 0x01 is the fractional when multiplied by 0.5 later on
	    	 fraction = (buf[0] & 0x01);


		    			// Fuck converting to 2's complement, since temperature never can be negative
/*			if ( val > 0x7F )
			{
				val |= 0x07F;
			}
*/
	            // Convert to float temperature value (Celsius)

	    	sprintf((char*)buf,"Temperature is %d.%d C\r\n", val, fraction * 5);
	    	    // Send out buffer (temperature or error message)
	     }// end else from i2c successful
		    	// Send out buffer either temperature or error message
	     HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	     HAL_Delay(500);	   // Wait
	}

	return ret;
}
/******************************************************************************************************
 A small 0.91" OLED display I bought from Amazon as a demo for the project
 *****************************************************************************************************/
void init_OLED(void)
{
#define COMMAND_REG	0x81	//
	// Fundamental Command Table
#define	ENTIRE_DISPLAY_ON	0xA4	// Resume to RAM content display (RESET)	Output follows RAM content
#define	DISPLAY_ON_BLAMK	0xA5	// Entire display ON	Output ignores RAM content
#define	SET_NORMAL_DISPLAY	0xA6	// Normal display (RESET)	0 in RAM: OFF in display panel	1 in RAM: ON in display panel
#define	SET_INVERSE_DISPLAY	0xA7	// Inverse display	0 in RAM: ON in display panel	1 in RAM: OFF in display panel
#define	DISPLAY_OFF			0xAE	// Display OFF (sleep mode)
#define	DISPLAY_ON			0xAF	// Display ON in normal mode
	// Scrolling Command Table
#define RIGHT_HORIZ_SCROLL	0x26	// Continuous Right Horizontal Scroll six more data bytes follow as scroll parameters
#define	LEFT_HORIZ_SCROLL	0x26	// Continuous Left Horizontal Scroll six more data bytes follow as scroll parameters
#define	STOP_SCROLLING		0x2E	// Stop Scrolling as configured by 0x27/0x27/0x29/0x2A
}

