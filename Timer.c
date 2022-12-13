/*
 * Timer.c
 *
 *  Created on: Aug 16, 2022
 *      Author: msmith
 */
//#include "usbd_cdc_if.h"	// Req'd to use the Timer init functions
//#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"		// Necessary to recognize TIM_HandleTypeDef htim1
#include "train.h"				// General train project #defines
#include <stdbool.h>
//#include "stm32f1xx_hal_tim.h"	// Not Req'd to use
extern TIM_HandleTypeDef htim1, htim2, htim3;
extern ADC_HandleTypeDef hadc1;
extern struct newCommand commandData;
extern int emble_speed_cmd();
//static unsigned int idleFIFO[MAX_FIFO_DEPTH];
static unsigned int buffInUse;
//bool update_Velocity_Display = true;
bool display_direction = true;
bool current_direction = true;
int knob_direction, debounce_count, last_direction;
int *cmdPtr;
//extern fwd_rev_button;	// bool rotary_sw_moved,
volatile static int clkPhase = 1, bitCounter, CmdRepeat, switch_Low_debounce_cnt, switch_high_debounce_cnt;//, old_direction;
extern int assemble_speed_cmd(void );
extern bool ADC_conversion_complete;
/*******************************************************************************
 * Definitions
 * RCC : Reset & Clock Control
 * ETR : External Trigger
 ******************************************************************************/
void timer_constructor(void)
{
	  // Create the idle packet
	for(int  i = 0; i < 49; i++)// Prior to init'ing Timer 1. It uses this FIFO for the first interrupt
	{
		commandData.cmdBuff[CMD_FIFO][i] = ZERO_HALF_PERIOD;		// Make them all non-preambles
		commandData.cmdBuff[IDLE_FIFO][i] = ZERO_HALF_PERIOD;		// This is the "idle" FIFO
	}
	buffInUse = 0;
	clkPhase = 1;
	commandData.NewDCC_Data = false;		// Let the system know we need to format a new command
	current_direction = true;
	cmdPtr = commandData.cmdBuff[IDLE_FIFO];	// restart pointer at the beginning of the idle command buffer
}
/*******************************************************************************
  * 			Timer 1 (TIM1) Interrupt Handler Function
  * Timer 1 takes care of putting the individual bit values out on to the track. If
  * in phase 1 continue with the value phase 2 preloaded into the "Auto-ReloadRegister".
  * If in phase 2, the timer continues with the value loaded last time but looks
  * for the next "bit" value and loads that value into the "Auto-ReloadRegister".
  *
  * Timer 1 does no calculations, it just feeds the next value into the
  * "Auto-ReloadRegister". When we've reached the end of the "command" FIFO we
  * ping-pong to the "idle" FIFO providing more than the 5mSec (8.8 mSec actually)
  * required by The NMRA spec S-9.2 footnote 11 .
  * 	"only that care must be taken to ensure that two packets with identical
  * 	addresses are not are not transmitted within 5 milliseconds of each other
  * 	for addresses in the range between 112-127 as older decoders may interpret
  * 	these packets as service mode packets (see RP-9.2.3).
  *
  * By ping-ponging between the "command" FIFO and the "idle" FIFO we transmit the
  * command as frequently as possible (S-9.2 Section C).
  *
  * Phase 1 lasts as long as it takes to empty one FIFO. Each FIFO ( instruction or idle )
  * takes (ONE_PULSE * 2) * MAX_FIFO_DEPTH or ( 58uSec * 2 ) * 44 = 5.104 mSec.
  * While the function assemble_speed_cmd() takes 53 uSecs from start to finish
  * as measured on the O'scope. Not to be confused with the 58uSec "one" period.
  * Therefore, phase one is the best place to run the function assemble_speed_cmd()
  * since pase 2 is writing data we may be altering in function assemble_speed_cmd().
 *******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{																	// I couldn't find idleFIFO in the old project
	// For every single half bit time we come here ( 58 or 100 uSec )
	if( htim == &htim1)			// Timer 1 produces the 58 & 100uSec DCC Bit Timing
	{
		  if( clkPhase == 1)				// First half of clock waveform
		  {
			  bitCounter++;
			  	  	  	  // if we exceeded bits left in this command, get the next command
			  if(bitCounter == MAX_FIFO_DEPTH )	// if we exceeded bits left in this command array, get the next command array
			  {
				  // Switch buffers and start pointer at offset zero '0'
				  if(buffInUse == IDLE_FIFO)		// #define IDLE_FIFO 1
				  {
					  buffInUse = CMD_FIFO;			// #define CMD_FIFO	 0
					  cmdPtr = commandData.cmdBuff[CMD_FIFO];	// restart pointer at the beginning of the new command
				  }
				  else
				  {
					  cmdPtr = commandData.cmdBuff[IDLE_FIFO];	// restart pointer at the beginning of the idle command
					  buffInUse = IDLE_FIFO;			// #define CMD_FIFO	 0
					  assemble_speed_cmd();						// Since IDLE_FIFO is in use we can overwrite CMD_FIFO
				  }												// assemble_speed_cmd() takes 53 uSecs from start to finish as measured on the O'scope.

				  // reguardless of which command array/buffer was chosen, start bit counter over again
				  bitCounter = 0;				// start at the beginning of the command array
			  }
			  clkPhase=2;			// next time around we execute phase 2
		  } // end if( clkPhase == 1)

			// Here we enter phase 2 of the clock. The period doesn't change here but we load the next period value to take
			// effect in phase 1

		  else	// clkPhase == 2. It's the second half of the waveform where we make any changes
		  {		// in the preload register that will appear in phase 0
			  TIM1->ARR = *cmdPtr++;	// Change the auto-reload value which takes effect next positive clock cycle
			  clkPhase = 1;				// phase 2 complete
		  }
	}// finished htim1 - Timer 1
/************************************************************************************
* 			Timer 2 (TIM2) Interrupt Handler Function
* Provides a 100 MSec timer to do an ADC conversion.
**************************************************************************************/
/*********************************************************************************************************************
* 							Speed rotary knob debounce (Timer 2 - 5 mSecs)
*  When relaxed (not switching) both of the switches are high.
* When turned clockwise yellow (CLK) goes low before blue (DT) never measured less than 16.00 mSec later.
**********************************************************************************************************************/
	else if ( htim == &htim2)	// Timer 2 issues cmds to the tracks every 5 mSecs
	{

			ADC_conversion_complete = false;
			HAL_ADC_Start_IT(&hadc1);
	}
/*********************************************************************************************************************
* 							Direction buttom/switch press debounce (Timer 2 purpose 2)
* Uses the 100 MSec timer to "debounce" the direction buttom/switc. If the direction buttom/switch is in the same state
* twice in a row, consider it debounced. It's not a momentary switch and 100 mSec's is sufficent.
**********************************************************************************************************************/
		else
		{
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0)//		If switch is at zero volts
			{
				switch_Low_debounce_cnt++;
				switch_high_debounce_cnt = 0;
				if(switch_Low_debounce_cnt >= 2 )	//  100mSec or 1/10 second
				{
					commandData.direction = !commandData.direction;	// Toggle current direction
					commandData.NewDCC_Data = true;		// Let the system know we need to format a new command
					display_direction = true;
				}
			}
			else	// Otherwise the switch has been released or was never touched.
			{
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1)//		If switch is at zero volts
				{
					switch_Low_debounce_cnt = 0;
					switch_high_debounce_cnt++;
					if(switch_high_debounce_cnt >= 2 )	//  100mSec or 1/10 second
					{
						switch_high_debounce_cnt = 0;
						commandData.direction = !commandData.direction;	// Toggle current direction
						commandData.NewDCC_Data = true;		// Let the system know we need to format a new command
						display_direction = true;
					}
				}
		}
} //  end else if this is Timer 2
	/************************************************************************************
	* 			Timer 3 (TIM3) Interrupt Handler Function
	* Timer 3s not used.
	**************************************************************************************/
}
