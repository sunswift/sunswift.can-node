/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: The template main file. It provides a simple example of using
                 some standard scandal functions such as the UART library, the
                 CAN library, timers, LEDs, GPIOs.
                 It is designed to compile and work for the 3 micros we use on
                 the car currently, the MSP430F149 and MCP2515 combination and
                 the LPC11C14 and LPC1768 which have built in CAN controllers.

                 UART_printf is provided by the Scandal stdio library and 
                 should work well on all 3 micros.

                 If you are using this as the base for a new project, you
                 should first change the name of this file to the name of
                 your project, and then in the top level makefile, you should
                 change the CHIP and MAIN_NAME variables to correspond to
                 your hardware.

                 Don't be scared of Scandal. Dig into the code and try to
                 understand what's going on. If you think of an improvement
                 to any of the functions, then go ahead and implement it.
                 However, before committing the changes to the Scandal repo,
                 you should discuss with someone else to ensure that what 
                 you've done is a good thing ;-)

                 Keep in mind that this code is live to the public on
                 Google Code. No profanity in comments please!

    Copyright (C) Etienne Le Sueur, 2011

    Created: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Template project
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef 	DOUBLE_BUFFER_EXAMPLE
#undef 	IN_CHANNEL_EXAMPLE
#undef WAVESCULPTOR_EXAMPLE

#define BASE_YEL_LED_PORT 3
#define BASE_YEL_LED_BIT 1
#define BASE_RED_LED_PORT 3
#define BASE_RED_LED_BIT 0


//#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>
#include <scandal/wavesculptor.h>
#include <scandal/error.h>

#include <string.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#include <arch/wdt.h>

void serial_send_ascii(u32 id, u08* buf, u08 length);

/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */
void setup(void) {

    WDT_Init(WATCHDOG_TIMER_PERIOD);
    
    int i=0;;
	GPIO_Init();
	GPIO_SetDir(BASE_RED_LED_PORT, BASE_RED_LED_BIT, OUTPUT);
	GPIO_SetDir(BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, OUTPUT);
    
    
    
     /* Initialise UART0 */
	UART_Init(115200);
    
    init_can();
    sc_init_timer();
    
    while(can_register_id(0, 0, 0, 1)==NO_ERR)
        ;

	/* Initialise Scandal, registers for the config messages, timesync messages and in channels */
	//scandal_init();
    
    

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);
    
    
    
}



/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {
    can_msg msg;
    uint8_t err;
    uint8_t txnum;
    uint8_t txbuf[20];
    uint8_t i;
    uint8_t chksum;
    uint8_t variableHold;
    
    //int i;
    
    err = can_get_msg(&msg);

	setup();
    
   
	/* Display welcome header over UART */
	//UART_printf("Welcome to the template project! This is coming out over UART0\n\r");
	//UART_printf("A debug LED should blink at a rate of 1HZ\n\r");

	sc_time_t one_sec_timer = sc_get_timer();


	/* This is the main loop, go for ever! */
	while (1) {
        err = can_get_msg(&msg);
        
        if(err == NO_ERR){
            serial_send_ascii(msg.id, msg.data, msg.length);
            
            /* RAW Mode */
            /*
            txnum=0;
            chksum=0;
            
            txbuf[txnum++]='C';
            variableHold = ((msg.id >> 24) & 0xFF);
            txbuf[txnum++] = variableHold;
            chksum += variableHold;
            
            variableHold = ((msg.id >> 16) & 0xFF);
            txbuf[txnum++] = variableHold;
            chksum += variableHold;
            
            variableHold = ((msg.id >> 8) & 0xFF);
            txbuf[txnum++] = variableHold;
            chksum += variableHold;
            
            variableHold = ((msg.id >> 0) & 0xFF);
            txbuf[txnum++] = variableHold;
            chksum += variableHold;
            
            for(i=0; i<8; i++){
                txbuf[txnum++]= msg.data[i];   
                 msg.data[i];              
            }
            
			txbuf[txnum++] = msg.length;
            chksum += msg.length;
            
            txbuf[txnum++] = chksum;
            //chksum 
            
            
            UART_Send(txbuf, txnum);
            */
        }

		/* Send a UART and CAN message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer) {
			/* Send the message */
			//UART_printf("1 second timer %u\n\r", (unsigned int)sc_get_timer());

			/* Send a channel message with a blerg value at low priority on channel 0 */
			scandal_send_channel(TELEM_LOW, /* priority */
									0,      /* channel num */
									0xaa   /* value */
			);
			/* Twiddle the LEDs */
			toggle_red_led();

			/* Update the timer to run 1000ms later */
			one_sec_timer = one_sec_timer + 1000;
		}
        
        WDT_Feed();
	}
}

void serial_send_ascii(u32 id, u08* buf, u08 length){
	int value;
	u32 uvalue = 0; 
	u08	i;
	
	value = (id >> 18) & 0xFF;
	switch(value){
    case CHANNEL_TYPE:
		UART_printf("C:\t");
		
		value = (id >> 26) & 0x07;
        //value = 7;
		UART_printf("%d", value);
        UART_printf("\t");
		
		value = (id >> 18) & 0xFF;
		UART_printf("%d", value); UART_printf("\t");
		
		value = (id >> 10) & 0xFF;
		UART_printf("%d", value); UART_printf("     ");
		
		value = (id >> 0) & 0x03FF;
		UART_printf("%d", value); UART_printf("\t");
		
		value = (u32)buf[0] << 24;
		value |= (u32)buf[1] << 16;
		value |= (u32)buf[2] << 8;
		value |= (u32)buf[3] << 0;
		UART_printf("%d", value); UART_printf("\t");
		
		uvalue = (u32)buf[4] << 24;
		uvalue |= (u32)buf[5] << 16;
		uvalue |= (u32)buf[6] << 8;
		uvalue |= (u32)buf[7] << 0;
		
		UART_printf("%u", value); // Time is unsigned
		UART_printf("\r\n");
		
		break;
		
	default:
		UART_printf("other\r\n");
		break;
	}
}

