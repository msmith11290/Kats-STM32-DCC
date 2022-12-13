/*
 * train.h
 *
 *  Created on: Aug 19, 2022
 *      Author: msmit
 */

#ifndef SRC_TRAIN_H_
//#define SRC_TRAIN_H_

// general definitions used by multiple "C" files
////#define TEST_PIN    16+-+++
#define false				0
#define true				1
#define CMD_FIFO			0
#define	IDLE_FIFO			1
#define	CLOCKWISE			0
#define COUNTER_CLOCKWISE	1
#define MAX_FIFO_DEPTH     	44
//#define	SWITCH_PRESSED		1
//#define REVERSE_SWITCH      4
//#define SOME_PIN            12  // Use GPIO number not pin number
//#define ANOTHER_PIN         13  //18  // (Pin 24) One of two H-Bridge polarity selectors
//#define H_BRIDGE_IN1        14  //19  // (Pin 25) One of two H-Bridge polarity selectors
//#define H_BRIDGE_IN2        15  //22  // PWM Signal to the motor
//#define SCOPE_TRIGGER_PIN   16


//#define ENCODER_ONE_BIT    	1
//#define ENCODER_ZERO_BIT   	0
#define SIZE_OF_PREAMBLE   	16
//#define PREAMBLE           	0x3FFF  // 11 1111 1111 1111, 14 one bits
//#define ACCESSORY_COMMAND  	0x80    // Bit 7 indicates an accessory command
#define SPEED_COMMAND      	0x40    // Bit 6 = indicate speed vs. accessory command
#define DIRECTION_BIT      	0x20    // Bit 5 indicates direction. Forward = 0x20, Reverse = 0x00
//#define REVERSE				0x00
//#define FORWARD				0x20
//#define SPEED_LSB          	0x10    // Bit 4 is the LSBit of speed. Strange I realize !?
#define ADDR_TOO_HIGH      	-100    // S-9.2 para B:Baseline Packets
//#define MAX_COMMANDS       	2       // Based on
//#define DCC_MODE           	1       // versus old school DC Mode
//#define SPEED_HALF_MAX     	4096/2  // 2048
#define ZERO_HALF_PERIOD   	100     // micro-Seconds
#define ONE_HALF_PERIOD    	58      // micro-Seconds
#define ZERO_PULSE   		100     // micro-Seconds
#define ONE_PULSE    		58      // micro-Seconds
#define PACKET_START_BIT   	ZERO_PULSE
#define DATA_BYTE_START_BIT ZERO_PULSE
#define PACKET_END_BIT     	ONE_PULSE
//#define MAX_ZERO_PERIOD    	12000   // micro-Seconds or 12 mSec's
//#define IDLE_PULSE         	ZERO_HALF_PERIOD // Make sure its at least a "zero" pulse so we're not preambling
//#define COMMAND_INTERVAL   	0.25/(ZERO_HALF_PERIOD * 0.000002)   // Convert two ZERO_HALF_PERIOD's to uSec's
//#define TIMER_ALARM_0      	0       // 4 alarms are available, I'm guessing "0" is the first
//#define IDLE_MODE          	true    // This is the mode we are in most of the time
//#define COMMAND_MODE       	false   // This mode is only used when changing speed, direction etc.
//#define DIR_SHIFT_BITS    	5
//#define DCC_STOP           	0
//#define	BUTTON_PRESSED		0
//#define GPIO_4             4       // Pull up to direction switch

// Command to and from PC
//#define POLL				1	// Inquiry, is there a DCC Controller at this Comm Port?
//#define	POLL_ACKNOWLEDGE	2	// Response to POLL - Yes, I am at this Comm Port (see POLL)
//#define	SPEED				3
//#define CMD_ACKNOWLEDGE		4	// General Acknowledge
//#define DIRECTION			5	// Change locomotive direction

		// To make sure the data as it changes follows us around without a million variables to keep track of
struct newCommand{
		unsigned char speed;// Locomotive speed/throttle setting
		unsigned char addr;	// Locomotive address
		_Bool	direction;	// forward or reverse. Specific value irrelevant, dpends on how train is placed on track
		_Bool	NewDCC_Data;// Something has changed and data is ready to be formatted into a DCC command
	//	_Bool	HW_Command;	// TRUE if command originated from hardware(HW), knobs, buttons etc. not from PC via USB
		_Bool	NewCmdRdy;	// A new command has been formated and is ready to be transmitted
		_Bool	PC_Cmd_Recvd;	// TRUE if a new command originated from PC via USB
		unsigned int ADC_result;
		int 	cmdBuff[2][MAX_FIFO_DEPTH];	// contains two commands, one currently in use and another being worked on
		int		buffInUse;	// Which of the two cmd[2][XX]'s currently in use
		};

#endif /* SRC_TRAIN_H_ */
