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

#define TORQUE_MODE_REGEN   0
#define TORQUE_MODE_OPEN    1
#define TORQUE_MODE_LIMSLIP 2
#define TORQUE_MODE_VECTOR  3


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
#include <project/ansi_color.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#include <arch/wdt.h>
void std_msg_handler(can_msg *msg);
float targetTorque(int32_t targetRPM, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct);
int32_t torqueControl(uint8_t controlMode, int32_t targetRPM, float desiredTorque, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct);
float absoluteFloat(float input);
float limit(float input, float limitAmount);
Wavesculptor_Output_Struct Left_WS;
Wavesculptor_Output_Struct Right_WS;


/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */
void setup(void) {

    //WDT_Init(WATCHDOG_TIMER_PERIOD);
    
    int i=0;;
	GPIO_Init();
	GPIO_SetDir(BASE_RED_LED_PORT, BASE_RED_LED_BIT, OUTPUT);
	GPIO_SetDir(BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, OUTPUT);
    
    
    
     /* Initialise UART0 */
	UART_Init(115200);
    
    //init_can();
    //sc_init_timer();
    
    register_standard_message_handler(std_msg_handler);
    
    scandal_init();
    
    can_register_id(0x7FF, 0x403, 0, 0);
    can_register_id(0x7FF, 0x423, 0, 0);
    /* TODO: Change this to only set up a finite number of message objects maybe 10? */
    while(can_register_id(0, 0, 0, 0)==NO_ERR)
        ;
    //can_register_id(0, 0, 0, 0);
    //can_register_id(0, 0, 0, 0);
    //can_register_id(0, 0, 0, 0);
    //can_register_id(0, 0, 0, 0);
	/* Initialise Scandal, registers for the config messages, timesync messages and in channels */
	
    
    

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);
    
    Left_WS.BaseAddress  = 0x400;
    Right_WS.BaseAddress = 0x420;
    
    Left_WS.ControlAddress  = 0x500;
    Right_WS.ControlAddress = 0x520;
    
    //scandal_send_ws_drive_command(0x523, 0, 0);
    
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
    float controllerOutput = 0;
    
    //int i;
    
    //err = can_get_msg(&msg);

	setup();
  
    
   
	/* Display welcome header over UART */
	//UART_printf("Welcome to the template project! This is coming out over UART0\n\r");
	//UART_printf("A debug LED should blink at a rate of 1HZ\n\r");

	sc_time_t one_sec_timer = sc_get_timer();


	/* This is the main loop, go for ever! */
    while (1) {
        handle_scandal();

		/* Send a UART and CAN message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer) {
			/* Send the message */
			UART_printf("\r\n"ANSI_YELLOW"Left Motor: %x \t Vel: %f \t LastUpd: %u \n\r"ANSI_RESET, Left_WS.BaseAddress, Left_WS.MotorVelocity, (unsigned int) Left_WS.lastUpdateTime);
      UART_printf(ANSI_YELLOW"Right Motor: %x \t Vel: %f \t LastUpd: %u\n\r"ANSI_RESET, Right_WS.BaseAddress, Right_WS.MotorVelocity, (unsigned int) Right_WS.lastUpdateTime);
      
      if(check_device_valid(&Left_WS)/*&check_device_valid(&Right_WS)*/){
        UART_printf(ANSI_YELLOW"\nValid\t T=%u\n"ANSI_RESET, sc_get_timer());
        controllerOutput = targetTorque(150, &Left_WS, &Right_WS);
        torqueControl(TORQUE_MODE_OPEN, 150, controllerOutput, &Left_WS, &Right_WS);
      }else{
        UART_printf(ANSI_RED"\nStale\t T=%u\n"ANSI_RESET, sc_get_timer());
      }
      
      if (controllerOutput < 0){
      UART_printf("NEG ");
      }
      //UART_printf(ANSI_YELLOW"CTRL = %1.3f, T=%u\n"ANSI_RESET, controllerOutput, sc_get_timer());
      
      //send_ws_drive_commands((float)120, (float)0.3, (float)0.3, &Left_WS);
      //send_ws_drive_commands((float)30, (float)0.3, (float)0.3, &Right_WS);
			
			/* Twiddle the LEDs */
			toggle_red_led();

			/* Update the timer to run 1000ms later */
			one_sec_timer = one_sec_timer + 100;
		}
        
        //WDT_Feed();
	}
}

void std_msg_handler(can_msg *msg){
    //UART_printf("STD_RCVD %d id= %d\r\n", sc_get_timer(), msg->id-0x420);
    uint16_t baseAddress = msg->id & 0x7E0;
    
    if(baseAddress == Left_WS.BaseAddress){
            scandal_store_ws_message(msg, &Left_WS);
    }else if(baseAddress == Right_WS.BaseAddress){
            scandal_store_ws_message(msg, &Right_WS);
    }else{
        UART_printf("Daaam, got some other message I wasn't expecting :S ID = 0x%x\r\n", (unsigned int) msg->id);
    }
    //scandal_store_ws_message(msg, &Left_WS);
}

/* Rename to calculateTorque */
float targetTorque(int32_t targetRPM, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct){
  #define FIXED_POINT_MULT  1000000
  #define PROPORTIONAL_MAX  0.3*FIXED_POINT_MULT
  #define INTEGRAL_MAX      0.2*FIXED_POINT_MULT
  int32_t leftVel; //angular velocity of the LHS wheel (rpm)
  int32_t rightVel; //angular velocity of the RHS wheel (rpm)
  int32_t avgVel;
  int error;
  int proportinalVar;
  static int integralVar=0;
  float output;
  
  leftVel = (ws_1_struct->MotorVelocity) * FIXED_POINT_MULT;
  rightVel = (ws_2_struct->MotorVelocity) * FIXED_POINT_MULT;
  avgVel = (leftVel + rightVel)/2;
  error = (targetRPM * FIXED_POINT_MULT) - avgVel; //Difference multiplied by FIXED_POINT_MULTI
  UART_printf("LV = %d, RV = %d, AV = %d, ER = %d\n\r", leftVel, rightVel, avgVel, error>>10);
  proportinalVar = (error >> 10);
  integralVar = integralVar + (error >> 16);

  if (proportinalVar > PROPORTIONAL_MAX){
    proportinalVar = PROPORTIONAL_MAX;
  } else if (proportinalVar < (-PROPORTIONAL_MAX)){
    proportinalVar = -PROPORTIONAL_MAX;
  }
  
  if (integralVar > INTEGRAL_MAX){
    integralVar = INTEGRAL_MAX;
  } else if (integralVar < (-INTEGRAL_MAX)){
    integralVar = -INTEGRAL_MAX;
  }

  output = (float) proportinalVar + (float) integralVar;
  //UART_printf("OUT = %f\n\r", output);
  //UART_printf("RV = %d ,PV = %d, IV = %d, OUT = %f\n\r", (int) rightVel, (int) proportinalVar, (int) integralVar, (float) output);

  output = output / FIXED_POINT_MULT;
  return output;
}

/* Pass targetRPM and desiredTorque, regen if targetRPM and desiredTorque sign different */
int32_t torqueControl(uint8_t controlMode, int32_t targetRPM, float desiredTorque, Wavesculptor_Output_Struct *ws_l_struct, Wavesculptor_Output_Struct *ws_r_struct){
  #define WS20_SPEED_LIMIT  100
  #define WS22_SPEED_LIMIT  400
  float WS20_Speed;
  float W22_Speed;
  int32_t deviceType;
  float sign=1;
  float leftTorque;
  float rightTorque;
  float slip;
  
  /* Note: Slip ratio can be calculated by (LeftSpeed / (LeftSpeed + RightSpeed)) or vice versa 0:All right, 0.5:Balanced ,1:All Left */
  
  if ( (controlMode == TORQUE_MODE_REGEN) | (controlMode == TORQUE_MODE_OPEN) ){
  
    leftTorque = absoluteFloat(desiredTorque);
    rightTorque = absoluteFloat(desiredTorque);
    
    /* Sign = sign of targetRPM if Sign of targetRPM and desiredTorque are different, regen */
    if(targetRPM > 0){
      sign = 1;
      if (desiredTorque > 0){
        /* Positive RPM and Torque = forward acceleration */
      } else {
        /* Positive RPM and negative Torque = forward regen */
          controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }
    }else{
      sign = -1;
      if (desiredTorque > 0){
        /* Negative RPM and positive Torque = reverse regen */
          controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }else{
        /* Negative RPM and negative Torque = reverse acceleration */
      }
    }
    
  } else if(controlMode == TORQUE_MODE_LIMSLIP){
    /* Implement me!! */
    
    if((ws_l_struct->MotorVelocity == 0.0) & (ws_r_struct->MotorVelocity == 0.0)){
      slip = 0.5;
    } else {
      slip = (absoluteFloat((ws_r_struct->MotorVelocity))/(absoluteFloat((ws_l_struct->MotorVelocity))+absoluteFloat((ws_r_struct->MotorVelocity))));
    }
    
    leftTorque = 2*slip*absoluteFloat(desiredTorque);
    leftTorque = limit(leftTorque, 0.5);
    rightTorque = 2*(1-slip)*absoluteFloat(desiredTorque);
    rightTorque = limit(rightTorque, 0.5);
    
    UART_printf("LT = %f\tRT = %f\n", leftTorque*1000, rightTorque*1000);
    
    /* Sign = sign of targetRPM if Sign of targetRPM and desiredTorque are different, regen */
    if(targetRPM > 0){
      sign = 1;
      if (desiredTorque > 0){
        /* Positive RPM and Torque = forward acceleration */
      } else {
        /* Positive RPM and negative Torque = forward regen */
          controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }
    }else{
      sign = -1;
      if (desiredTorque > 0){
        /* Negative RPM and positive Torque = reverse regen */
          controlMode = TORQUE_MODE_REGEN; //So we set the speed to zero later to regen
      }else{
        /* Negative RPM and negative Torque = reverse acceleration */
      }
    }
  }
  
  /* Setting set speed limits to wavesculptor types */
  if ( controlMode == TORQUE_MODE_REGEN){
    WS20_Speed = 0;
    W22_Speed = 0;
  } else {
    WS20_Speed = WS20_SPEED_LIMIT;
    W22_Speed = WS22_SPEED_LIMIT;
  }
  
    
  /* Set motor speed limit depending on type of motor controller, WS22 is in RPM, WS20 is in m/s - send roughly RPM/4 */
  deviceType = check_device_type(ws_l_struct);
  
  if(deviceType == WS_22){
    send_ws_drive_commands(sign*W22_Speed, leftTorque, (float) 1.0, ws_l_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_Speed, leftTorque, (float) 1.0, ws_l_struct);
  }


  deviceType = check_device_type(ws_r_struct);

  if(deviceType == WS_22){
    send_ws_drive_commands(sign*W22_Speed, rightTorque, (float) 1.0, ws_r_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_Speed, rightTorque, (float) 1.0, ws_r_struct);
  }

  /* Send command to each WS: send_ws_drive_commands(sign*MAX_SPEED, torque_command, (float)0.3, ws_1_struct); */
  return 0;
}

float absoluteFloat(float input){
  
  if(input > 0){
    return input;
  } else if (input < 0) {
    return (-1.0*input);
  }
  
  return 0.0;
}

float limit(float input, float limitAmount){
  if(input > limitAmount){
    input = limitAmount;
  }
  return input;
}