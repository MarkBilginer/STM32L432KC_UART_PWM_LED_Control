/*
 * pwm_app.c
 *
 *  Created on: 28 May 2021
 *      Author: mark
 */


#include "pwm_app.h"


void setup_micro(){
	init_hal();
	config_system_clock();
	  init_gpio();
	  init_usart2_uart(115200);
	  init_tim1(999, 199, 100);
	  init_tim2(999, 199, 100);
}


void start_pwm(){
	  complement_tim_pwm_start();
	  tim_pwm_start();
}


void start_application(){

	  credential_print();

	  print_led_prompt();

	  receive_led_input();

	  int step1_success = which_led_print();

	  if(step1_success){

		  print_duty_cycle_prompt();

		  int step2_success = receive_duty_cycle();

		  if(step2_success){
			  led_percentage_print();

			  dim_chosen_led();
		  }

	  }

}
