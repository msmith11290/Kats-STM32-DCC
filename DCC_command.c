/*
 * Loco_command.c
 *
 *  Created on: Aug 10, 2022
 *      Author: msmit
 */
/******************************************************************************
 * Assemble a command while padding bits to allow the one's & zero's to meet
 * minimum timing requirements driven by cutout(Tcs) start from S-9.3.2 Section 2.4
 * All times are from zero startin point so some algebraic summing needs to be done
 * ***************************************************************************
Tcs     1           // high for 26-32uSec cutout_start
Tts1    Tcs + 2     // Minimum Start (80 uSec) channel 1 after rise/start of Tcs
Tts2    3           // Maximum End channel 1
Ttc1                // 177 uSec

Single GPIO pulse    30uS
One bit     52uS <-> 64uS       Nominal 58uS or full sq wave = 17.2 KHz. 2 cycles = 8.63KHz
Zero bit    95uS <-> 9900uS     Nominal 100uS or full sq wave = 19KHz
Tcs         26uS <-> 32uSec     Nominal 29uS, 1/2 of a One bit = 29uS or 34.483 KHZ

Tcs requires a single pulse for the the first bit which means the FIFO must end on a
low cycle. One single pulse (30uS) makes Tcs. Follow with Tts1 - Tcs (80uS - 30uS) = 50uS
minimum or two more clock periods for 60uS. For Railcom I don't know if we enter a tri-state or go low
during this time ??? The timing diagram looks tristated and the data bits attenuated.

BTW : MSB shifted out first set by sm_config_set_out_shift in pio_serializer.pio
**********************************************************************************/
#include "stdio.h"          // Debug only, like printf()
#include <string.h>         // For memset(). Used to zero out cmd_fifo[]#include <stdio.h>
//#include "loco_command.h"
#include "train.h"
#include <stdbool.h>
#include "stm32f1xx_hal.h"		// Necessary to toggle GPIO's while testing

//void wrap( unsigned char value, int size );

unsigned int cmdFIFO[MAX_FIFO_DEPTH];  // 16 preambles, 24 data + 4 start/stop bits
//unsigned int idleFIFO[MAX_FIFO_DEPTH];
unsigned int forwardFIFO[MAX_FIFO_DEPTH];  // For debugging only. It's a hand assembled command
unsigned int reverseFIFO[MAX_FIFO_DEPTH];  // For debugging only. It's a hand assembled command
struct newCommand commandData;
extern bool new_command_available;
/**************************************************************************************
 * Data is stored as a timer value, one word. no bits.
 * a "1" : ONE_HALF_PERIOD    	58      // micro-Seconds
 * a "0" : ZERO_HALF_PERIOD   	100     // micro-Seconds
 * No fancy shifting data to reduce errors. We do right shift the mask. MSB to LSB
 * ************************************************************************************/
int storeToFIFO( unsigned char value, unsigned int FIFO_index)
{
    int i, mask;

    mask = 0x80;
    for( i = FIFO_index; mask > 0;  i++, mask >>= 1)     // 8 bits, 8 FIFO locations, MSB first
    {
       if( value & mask )
    	   cmdFIFO[FIFO_index++] = ONE_PULSE;
       else
           cmdFIFO[FIFO_index++] = ZERO_PULSE;
    }
    return i;
}
/**************************************************************************************
 *                      assemble_speed_command()
 * See S-9.2 Section A General Packet Format
 *
 * First add the preamble ( 14 bits )to the left most bits.S-9.2
 *
 * The "Packet Start Bit" is the first bit with a value of "0" that follows a valid preamble.
 *
 * The first data byte of the packet normally contains eight bits of address information.
 * The first bit is the MSB. For non-normal see 9.2.3.
 *
 * Data Byte Start Bit: This bit precedes a data byte and has the value of "0".
 * The error byte is the XOR of the address and data bytes. A Packet End Bit "1" follows
 * this byte.
 * 	 	 	 	 	 	 	  Real life timing issue
 * assemble_speed_cmd() takes 53 uSecs from start to finish as measured on the O'scope. .
 * Only an accidental relationship to the 58uSec "one" period.
 * Each FIFO ( instruction/idle ) takes (ONE_PULSE * 2) * MAX_FIFO_DEPTH or
 * ( 58uSec * 2 ) * 44 = 5.104 mSec.
 * ************************************************************************************/
int assemble_speed_cmd(void )
{
    unsigned char velocity = 0;
    int index, staleBuff;
    //unsigned char temp;

    for( index = 0; index < SIZE_OF_PREAMBLE; index++)
    	cmdFIFO[index] = ONE_PULSE;            // Sixteen ones. Minimum is 14

    cmdFIFO[index++] = PACKET_START_BIT;//  "zero" 100uSec's
    // Begin working on the 8 bit Address
    if(commandData.addr > 127)              // Only 7 bits of address are valid ? see S-9.2 Figure 1
    {
        while(1);
        return ADDR_TOO_HIGH;
    }
    else
    {
        index = storeToFIFO( commandData.addr, index);  // index s/b 0x11 or 17 decimal before execution
        cmdFIFO[index++] = DATA_BYTE_START_BIT;        // DATA_BYTE_START_BIT "zero" 100us, index = 0x19 or 25 decimal  before executing

    // Begin working on the Command Byte
        velocity = SPEED_COMMAND;
        velocity |= commandData.speed;		// debug only picked this to verify hand drawn command, uncomment the next 12 lines for real life
        if( commandData.direction )                     // If reverse, add 0x20 bit
            velocity |= DIRECTION_BIT;    // 0x60, bit 6 = 1, 5 == 1 for forward. Bits 7 = 0 & 6 = 1, 5 = 1 result=(0x60) == forward speed command
        index = storeToFIFO( velocity, index);
        cmdFIFO[index++] = DATA_BYTE_START_BIT;    // index s/b 34 at this point

    // Begin working on the Error check byte
        index = storeToFIFO( (commandData.addr ^ velocity), index);     // Error check byte, index s/b 35 at this point
        cmdFIFO[index++] = PACKET_END_BIT;        // Add the final item: Packet End Bit "1", index s/b 32 at this point end scenario 1 */
        while( index < MAX_FIFO_DEPTH )
        		cmdFIFO[index++] = ZERO_HALF_PERIOD;

    	// Figure out which one of two command FIFO's we want to use
        staleBuff = ( commandData.buffInUse == 0) ? 1 : 0;

        // For readability we used cmdFIFO. Now we must copy its contents into the working FIFO
        for( index = 0; index < MAX_FIFO_DEPTH; index++)
        	commandData.cmdBuff[staleBuff][index] = cmdFIFO[index];
        commandData.NewCmdRdy = true;
    }
	commandData.NewDCC_Data = false;
    return 1;	// success
}
/*
These value are ahnd entered to avoid any software issues. We ended up with
24 trues, 26 falses. 24 * (116uS * 2) = 2784us and 26 * (100uS * 2) = 5200uS
for a total of 2.784mS + 5.2 mS = 7.984mS command transmission time
*/
void reverse_test_init(void)
{
    reverseFIFO[0]= ONE_HALF_PERIOD;;
    reverseFIFO[1]= ONE_HALF_PERIOD;
    reverseFIFO[2]= ONE_HALF_PERIOD;
    reverseFIFO[3]= ONE_HALF_PERIOD;
    reverseFIFO[4]= ONE_HALF_PERIOD;
    reverseFIFO[5]= ONE_HALF_PERIOD;
    reverseFIFO[6]= ONE_HALF_PERIOD;
    reverseFIFO[7]= ONE_HALF_PERIOD;
    reverseFIFO[8]= ONE_HALF_PERIOD;
    reverseFIFO[9]= ONE_HALF_PERIOD;
    reverseFIFO[10]= ONE_HALF_PERIOD;
    reverseFIFO[11]= ONE_HALF_PERIOD;
    reverseFIFO[12]= ONE_HALF_PERIOD;
    reverseFIFO[13]= ONE_HALF_PERIOD;         // 14 ones are the minimum for a pre-amble as per spec
    reverseFIFO[14]= ONE_HALF_PERIOD;
    reverseFIFO[15]= ONE_HALF_PERIOD;         // 16 ones is what DCC++ uses, so I duplicated that
    reverseFIFO[16]= PACKET_START_BIT;  // Packet Start Bit "0"
    //					0000 0010 = 0x02
    reverseFIFO[17]= ZERO_HALF_PERIOD;         // Addr 7
    reverseFIFO[18]= ZERO_HALF_PERIOD;
    reverseFIFO[19]= ZERO_HALF_PERIOD;
    reverseFIFO[20]= ZERO_HALF_PERIOD;        // Addr 4
    reverseFIFO[21]= ZERO_HALF_PERIOD;         // Addr 3
    reverseFIFO[22]= ZERO_HALF_PERIOD;
    reverseFIFO[23]= ONE_HALF_PERIOD;         // Address = 2
    reverseFIFO[24]= ZERO_HALF_PERIOD;         // Addr 0
    reverseFIFO[25]= DATA_BYTE_START_BIT;  // Data Byte Start Bit "0"
    //					0100 0100 = 0x44
    reverseFIFO[26]= ZERO_HALF_PERIOD;         // Command 01XX XXXX is speed command
    reverseFIFO[27]= ONE_HALF_PERIOD;         // Command 01XX XXXX is speed command
    reverseFIFO[28]= ZERO_HALF_PERIOD;         // Direction bit(0x20). 1 = Forward, 0 = Reverse, depending on which way the train is facing
    reverseFIFO[29]= ZERO_HALF_PERIOD;         // Speed LSB 0
    reverseFIFO[30]= ZERO_HALF_PERIOD;         // Speed LSB 4
    reverseFIFO[31]= ONE_HALF_PERIOD;         // Speed LSB 3
    reverseFIFO[32]= ZERO_HALF_PERIOD;         // Speed LSB 2
    reverseFIFO[33]= ZERO_HALF_PERIOD;         // Speed LSB 1
    reverseFIFO[34]= DATA_BYTE_START_BIT;  // Data Byte Start Bit "0"
    //					0100 0110 = 0x46
    reverseFIFO[35]= ZERO_HALF_PERIOD;         // Error byte = speed byte XOR'd with Address byte 0x02 ^ 0x66 = 0x64
    reverseFIFO[36]= ONE_HALF_PERIOD;
    reverseFIFO[37]= ZERO_HALF_PERIOD;
    reverseFIFO[38]= ZERO_HALF_PERIOD;
    reverseFIFO[39]= ZERO_HALF_PERIOD;
    reverseFIFO[40]= ONE_HALF_PERIOD;
    reverseFIFO[41]= ONE_HALF_PERIOD;
    reverseFIFO[42]= ZERO_HALF_PERIOD;
    reverseFIFO[43]= PACKET_END_BIT; // Packet End Bit "1"
            // MAX_FIFO_DEPTH == 44
}
void forward_test_init(void)
{
    forwardFIFO[0]= ONE_HALF_PERIOD;
    forwardFIFO[1]= ONE_HALF_PERIOD;
    forwardFIFO[2]= ONE_HALF_PERIOD;
    forwardFIFO[3]= ONE_HALF_PERIOD;
    forwardFIFO[4]= ONE_HALF_PERIOD;
    forwardFIFO[5]= ONE_HALF_PERIOD;
    forwardFIFO[6]= ONE_HALF_PERIOD;
    forwardFIFO[7]= ONE_HALF_PERIOD;
    forwardFIFO[8]= ONE_HALF_PERIOD;
    forwardFIFO[9]= ONE_HALF_PERIOD;
    forwardFIFO[10]= ONE_HALF_PERIOD;
    forwardFIFO[11]= ONE_HALF_PERIOD;
    forwardFIFO[12]= ONE_HALF_PERIOD;
    forwardFIFO[13]= ONE_HALF_PERIOD;         // 14 ones are the minimum for a pre-amble as per spec
    forwardFIFO[14]= ONE_HALF_PERIOD;
    forwardFIFO[15]= ONE_HALF_PERIOD;         // 16 ones is what DCC++ uses, so I duplicated that
    forwardFIFO[16]= PACKET_START_BIT;  // Packet Start Bit "0"
    //				0000 0010 = 0x02
    forwardFIFO[17]= ZERO_HALF_PERIOD;         // Addr 7
    forwardFIFO[18]= ZERO_HALF_PERIOD;
    forwardFIFO[19]= ZERO_HALF_PERIOD;
    forwardFIFO[20]= ZERO_HALF_PERIOD;        // Addr 4
    forwardFIFO[21]= ZERO_HALF_PERIOD;         // Addr 3
    forwardFIFO[22]= ZERO_HALF_PERIOD;
    forwardFIFO[23]= ONE_HALF_PERIOD;         // Address = 2
    forwardFIFO[24]= ZERO_HALF_PERIOD;         // Addr 0
    forwardFIFO[25]= DATA_BYTE_START_BIT;  // Data Byte Start Bit "0"
    //				0110 0100 = 0x64
    forwardFIFO[26]= ZERO_HALF_PERIOD;         // Command 01XX XXXX is speed command
    forwardFIFO[27]= ONE_HALF_PERIOD;         // Command 01XX XXXX is speed command
    forwardFIFO[28]= ONE_HALF_PERIOD;         // Direction bit(0x20). 1 = Forward, 0 = Reverse, depending on which way the train is facing
    forwardFIFO[29]= ZERO_HALF_PERIOD;         // Speed LSB 0
    forwardFIFO[30]= ZERO_HALF_PERIOD;         // Speed LSB 4
    forwardFIFO[31]= ONE_HALF_PERIOD;         // Speed LSB 3
    forwardFIFO[32]= ZERO_HALF_PERIOD;         // Speed LSB 2
    forwardFIFO[33]= ZERO_HALF_PERIOD;         // Speed LSB 1
    forwardFIFO[34]= DATA_BYTE_START_BIT;  // Data Byte Start Bit "0"
    //				0110 0110 = 0x66
    forwardFIFO[35]= ZERO_HALF_PERIOD;         // Error byte = speed byte XOR'd with Address byte 0x02 ^ 0x66 = 0x64
    forwardFIFO[36]= ONE_HALF_PERIOD;
    forwardFIFO[37]= ONE_HALF_PERIOD;
    forwardFIFO[38]= ZERO_HALF_PERIOD;
    forwardFIFO[39]= ZERO_HALF_PERIOD;
    forwardFIFO[40]= ONE_HALF_PERIOD;
    forwardFIFO[41]= ONE_HALF_PERIOD;
    forwardFIFO[42]= ZERO_HALF_PERIOD;
    forwardFIFO[43]= PACKET_END_BIT; // Packet End Bit "1"
    // MAX_FIFO_DEPTH == 44
}


