/** @file pwm_app.c
 *  @brief Application layer helper and wrapper functions implementation.
 *
 *  Aims to make the code more readable. Application layer,
 *  library layer and driver layer concept was in mind.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */

/* -- Includes -- */
#include "pwm_app.h"

void setup_micro()
{
	init_hal();
	config_system_clock();
	init_gpio();
	init_usart2_uart(115200);
	init_tim1(999, 199, 100);
	init_tim2(999, 199, 100);
}

void start_pwm()
{
	complement_tim_pwm_start();
	tim_pwm_start();
}

void start_application()
{
	credential_print();

	print_led_prompt();
	int step1_success = receive_led_input();

	if (step1_success) {

		print_duty_cycle_prompt();
		int step2_success = receive_duty_cycle();

		if (step2_success) {
			print_led_percentage();
			dim_chosen_led();
		}

	}

}
