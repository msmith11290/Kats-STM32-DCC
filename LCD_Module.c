/*
 * LCD_Module.c
 *
 *  Created on: Sep 24, 2022
 *      Author: msmit
 */
#include "stm32f1xx_hal.h"	// Handy for everyday things like uint8_t
#include "i2c.h"
#define	LCD_ADDRESS	0x78	// 0x78 = (0x3C << 1 ); J8,J10 short,J7,J9open, SA1=0,SA0=0(default setting); Page 17 Figure 9.5

uint8_t lcdBuff[50];

extern I2C_HandleTypeDef hi2c1;
/*****************************1***************************************************************
 * InitRW1063() : Is taken direct from the LCD Specification page 19. The Fact that certian
 * bits are set in the data word determine which command you are issueing. This clear based
 * on section 12. Instruction Table of the manual. However, the very last instruction in the
 * I think this is where the data you want to display is controlled.
 *
 * There is some useful information on website : https://controllerstech.com/i2c-lcd-in-stm32/
 * I googled using : LCD Programming Example using i2c
 *******************************************************************************************/
//int InitRW1063(void)
int InitLCD_Module(void)
{
	int ret = 0;
	// WriteInst (0x38); //DL=1: 8 bits; N=1: 2 line; F=0: 5 x 8dots
lcdBuff[0] = 0x38;	// Function Set 8-bit, 2-line, 5x8 dots/char as per Digi-Key's product desciption
ret = HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, lcdBuff, ONE_DATA_BYTE, HAL_MAX_DELAY);

	// WriteInst (0x0c); // D=1, display on; C=B=0; cursor off; blinking off;
lcdBuff[0] = 0x0c;
ret = HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, lcdBuff, ONE_DATA_BYTE, HAL_MAX_DELAY);

	// WriteInst (0x06); // I/D=1: Increment by 1; S=0: No shift
lcdBuff[0] = 0x0c;
ret = HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, lcdBuff, TWO_DATA_BYTES, HAL_MAX_DELAY);
return ret;
}
/*
void WriteData(BYTE byData)
{
I2C_Start() - I2C_Send(0x78) - I2C_Ack()
I2C_Send(0x40);
I2C_Ack();
I2C_Send(byData);
I2C_Ack();
I2C_Stop();
}

void WriteInst(BYTE byInst)
{
I2C_Start();
I2C_Send(0x78);
I2C_Ack();
I2C_Send(0x00);
I2C_Ack();
I2C_Send(byInst);
I2C_Ack();
I2C_Stop();
}
*/
