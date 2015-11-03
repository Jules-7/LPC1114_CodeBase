/**************************************************************************/
/*! 
    @file     main.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2011, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "projectconfig.h"
#include "sysinit.h"
#include "LPC111x.h"                        /* LPC111x definitions */

#include <stdio.h>

#include "commands.h"       // Generic helper functions

#include "core/timer32/timer32.h"
#include "core/ssp/ssp.h"
#include "core/timer16/timer16.h"
#include "core/cmd/cmd.h"
#include "core/gpio/gpio.h"
#include "core/systick/systick.h"
//#include "core/arinc/arinc.h"

#include "drivers/fatfs/diskio.h"
#include "drivers/fatfs/ff.h"


/*********************************** DEFINES ******************************/
#define LISTEN_PORT 0	// Port for interrupts - where data comes in
#define LISTEN_PORT_BIT 5  // bit on port for interrupts - where data comes in
#define RED_LED_PORT 1		// Port for led
#define RED_LED_BIT 9		// Bit on port for led
#define ON  0  // Level to set port to turn on led
#define OFF  1  // Level to set port to turn off led

#define RED_LED_TOGGLE()  if((GPIO_GPIO1DATA & (1<<RED_LED_BIT))) \
                          RED_LED_on(); \
                      else          \
                          RED_LED_off()
#define RED_LED_on()  gpioSetValue( RED_LED_PORT, RED_LED_BIT, ON  )
#define RED_LED_off()  gpioSetValue( RED_LED_PORT, RED_LED_BIT, OFF )


#define GREEN_LED_PORT 3		// Port for led
#define GREEN_LED_BIT 0		// Bit on port for led

#define GREEN_LED_on()  gpioSetValue( GREEN_LED_PORT, GREEN_LED_BIT, ON  )
#define GREEN_LED_off()  gpioSetValue( GREEN_LED_PORT, GREEN_LED_BIT, OFF )


#define PACKAGE_SIZE  1024

//#define f_tell(fp) ((fp)->fptr)
/**************************************************************************/

/******************************* VARIABLES ********************************/
extern volatile uint32_t timer16_0_counter;
extern volatile uint32_t one_second_counter;

uint8_t ARINC = 0;
uint8_t size = 100;
//uint8_t max_value = 0;
uint32_t maximum = 0;
uint8_t t = 0;

// variables for fatfs operation
FRESULT res;
UINT BYtes_read=0;
FIL fil;       /* File object */
DWORD endOfFile = 0;
static FATFS Fatfs[1];


uint32_t data_array[100] = {0};  // just an array to record timer_counter value
                            // it allows us to get the length of each pulse

//counters
uint8_t j = 0;
uint8_t i = 0;
uint8_t p = 0;
uint8_t FLAG_OF_ONE = 0;
uint32_t BYTES_COUNTER = 0;
uint8_t BITS_COUNTER = 0;

//flags
uint8_t START_RECORD = 0;
uint8_t RECORD_PACKET = 0;
uint8_t GET_ZERO = 0;


uint32_t regVal = 0;
uint32_t DATA = 0;
uint8_t ARINC_PACKAGE [PACKAGE_SIZE] = {0};
uint8_t WRITE_PACKAGE [PACKAGE_SIZE] = {0};
uint8_t BYTE_to_write = 0;

/**************************************************************************/

int FindMaxValueInArray(void) {
	int maxValue = data_array[0];
	for (t = 1; t < size; ++t) {
		if ( data_array[t] > maxValue ) {
			maxValue = data_array[t];
		}
	}
	ARINC = 1;
	GET_ZERO = 1;
	memset(&data_array, 0, sizeof data_array);
	return maxValue - 10;
}

void PIOINT0_IRQHandler(void){
	// Gets the interrupt status for a specific port pin.
	// '1' if an interrupt was raised by the specified pin, otherwise '0'.
    regVal = gpioIntStatus(LISTEN_PORT, LISTEN_PORT_BIT);
    // check if value is not empty
	if (regVal){
		if (!GET_ZERO){  // this part is used as part of ARINC rate determination
			data_array[j] = timer16_0_counter;
			j++;
			if(j == 100){
				j = 0;
			}

		}
		else {
//			// TEST
//			DATA = timer16_0_counter;
//			ARINC_PACKAGE[BYTES_COUNTER] = DATA;
//
//			if (BYTES_COUNTER == PACKAGE_SIZE -1) {
//				RECORD_PACKET = 1;
//				memcpy(WRITE_PACKAGE, ARINC_PACKAGE, sizeof ARINC_PACKAGE);
//				memset(&ARINC_PACKAGE, 0, sizeof ARINC_PACKAGE);
//				BYTES_COUNTER = 0;
//				GREEN_LED_on();
//			} else {
//				BYTES_COUNTER ++;
//			}
//		}
//		// END TEST

			// the value of pulse length
			DATA = timer16_0_counter;

			if ((DATA >= maximum)&&(!START_RECORD)) {  // length is zero
				START_RECORD = 1;  // allow to record data
			}

			if (START_RECORD) {
                data_array[p] = DATA;
                p ++;
				if (DATA >= maximum){  // if length of zero
					BYTE_to_write &= ~(0x80>>i);  // write 0
					i++;
				}
				else {  // if length of one
					if (!FLAG_OF_ONE){  //if it is the first one
						BYTE_to_write |= (0x80>>i);  // write 1
						FLAG_OF_ONE = 1;  // raise flag true - so the next one is not recorded
						i++;
					}
					else {
						FLAG_OF_ONE = 0;
					}
				}
				if (i == 8){
					ARINC_PACKAGE[BYTES_COUNTER] = BYTE_to_write;

					BYTE_to_write = 0;
					i = 0;
					if (BYTES_COUNTER == PACKAGE_SIZE - 1){
						RECORD_PACKET = 1;
						memcpy(WRITE_PACKAGE, ARINC_PACKAGE, sizeof ARINC_PACKAGE);
						memset(&ARINC_PACKAGE, 0, sizeof ARINC_PACKAGE);
						BYTES_COUNTER = 0;
                        GREEN_LED_on();
                        memset(&data_array, 0, sizeof data_array);
                        p = 0;
					}
					else {
						BYTES_COUNTER ++;
					}
				}
			}
		}
		timer16_0_counter = 0;
		// clear interrupt or lower down the interrupt flag
		gpioIntClear(LISTEN_PORT, LISTEN_PORT_BIT);
	}
	return;
}


int main(void){
	/************************************* INITIALIZATION *************************************/

    systemInit();  // Configure cpu and mandatory peripherals

	gpioInit();  //Initialise gpio

    // Initialise Timer16_0 to tick at rate of 1/100000th of second.
   	// Note that as this is a 16 bit timer, the maximum count we can
   	// load into timer is 0xFFFF, or 65535. Default clock speed
   	// set up by CMSIS SystemInit function - SystemCoreClock - is
   	// 48MHz or 48000000 Hz. Dividing this by 2000 is 24000 which is
   	// within appropriate timer16 maximum. This could be extended
   	// if timer routine extended to make use of Prescale Counter register
   	// Note by default LPC_SYSCON->SYSAHBCLKDIV is 1.
/****************************************************************************************/
	char buffer[13]={'l','u','c','h','0','0','0','0','.','d','a','t','\0'};
	UINT br;

	res = f_mount(0, &Fatfs[0]);
	if (res != FR_OK){
	   return 0;
	}

	res = f_open (&fil, "info.dat", FA_READ | FA_WRITE | FA_OPEN_EXISTING);
	if (res) {
		res = f_open (&fil, "info.dat", FA_WRITE | FA_CREATE_NEW);
	}
	else {
		res = f_read (&fil, buffer, sizeof buffer, &br);

		if (buffer[7] == '9'){
			buffer[7] = '0';
			if (buffer[6] == '9'){
				buffer[6] = '0';
				if (buffer[5] == '9'){
					buffer[5] = '0';
					buffer[4]+=1;
				}
				else buffer[5]+=1;
			}
			else buffer[6]+=1;
		}
		else buffer[7]+=1;
		buffer[12] = '\0';

		res = f_lseek(&fil, endOfFile);
	}
	res = f_write (&fil, buffer, sizeof buffer, &br);

	f_close(&fil);
/****************************************************************************************/

	timer16Init(0, CFG_CPU_CCLK/10000);
	timer16_0_counter = 0;  // Initialise counter that counts Timer16_0 ticks
	timer16Enable(0);  //Enable Timer16_0

	gpioSetDir(RED_LED_PORT, RED_LED_BIT, 1);  // Set port for RED LED to output
	gpioSetDir(GREEN_LED_PORT, GREEN_LED_BIT, 1);  // Set port for GREEN LED to output

	gpioSetDir(LISTEN_PORT, LISTEN_PORT_BIT, gpioDirection_Input);  // Set GPIO0.5 to input
	gpioSetPullup (&IOCON_PIO0_5, gpioPullupMode_Inactive);  // Disable the internal pullup/down resistor on P0.5
	gpioSetInterrupt(  // Setup an interrupt on GPIO0.5
			         LISTEN_PORT,                     // Port
				     LISTEN_PORT_BIT,                 // Pin
				     gpioInterruptSense_Edge,         // Edge/Level Sensitive
				     gpioInterruptEdge_Double,        // Single/Double Edge
					 gpioInterruptEvent_ActiveLow);   // Rising/Falling

	gpioIntEnable(LISTEN_PORT, LISTEN_PORT_BIT);  // Enable the interrupt
	/****************************************************************************************/

// 		Try to mount drive
//    res = f_mount(0, &Fatfs[0]);
//    if (res != FR_OK){
//       return 0;
//    }

	/****************************************************************************************/
    while (1) {
	    if (!ARINC) {
		    if (data_array[98] != 0) {  // read data for one second - and determine arinc rate
		    	maximum = FindMaxValueInArray();
			}
		}

      if (RECORD_PACKET) {

    		res = f_open (&fil, buffer, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    		if (res) {
    			GREEN_LED_on();
    		}
    		res = f_lseek(&fil, (&fil)->fsize);
    		res = f_write (&fil, &WRITE_PACKAGE, PACKAGE_SIZE, &BYtes_read);
     		if (res) {
        		GREEN_LED_on();
        	}
    		f_close(&fil);
    		memset(&WRITE_PACKAGE, 0, sizeof WRITE_PACKAGE);
    		RED_LED_TOGGLE();
    		RECORD_PACKET = 0; //put flag down
    		GREEN_LED_off();
    		endOfFile += PACKAGE_SIZE;
      }
    }
}




