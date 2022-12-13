/*
 * i2c.h
 *
 *  Created on: Aug 7, 2022
 *      Author: msmit
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

	// IO_Expander
#define	LM75_ADDR 0x48 << 1 		// Use 8 bit address so, 7 bit addr shifted into bits 7..1
#define	PCF8575_ADDR	0x20 << 1	// PCF8575 IO Expander Board Module I2C to 16 I/O. Address bit 6 described in data sheet.
//#define POINTER_REG		0x00		// Pointer Register (Selects Which Registers Will Be Read From or Written to):

	// i2c_temperature
#define	TEMPERATURE_REG	0x00		// Register that has the temperature
//#define ID_REGISTER		0x07		// Product ID Register (Read-Only) Pointer Address: 07h

	// General Defines
#define ONE_DATA_BYTE	0x01		// 8 bit data value
#define TWO_DATA_BYTES	0x02		// 16 bit data value
#define BIT_0			0x01
#define BIT_1			0x02
#define BIT_2			0x04
#define BIT_3			0x08
#define BIT_4			0x10
#define BIT_5			0x20
#define BIT_6			0x40
#define BIT_7			0x80

#endif /* INC_I2C_H_ */
