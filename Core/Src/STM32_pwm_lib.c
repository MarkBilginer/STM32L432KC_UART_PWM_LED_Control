/** @file STM32_pwm_lib.c
 *  @brief Library layer function implementation for the STM32L4.
 *
 *  This file contains the applications' library abstraction layer, wrapping
 *  the driver layer functions which are vendor specific. Aims at making
 *  the code more portable.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */


#include "STM32_pwm_lib.h"

/* Handler for TIM1*/
TIM_HandleTypeDef htim1;

/* Handler for TIM2*/
TIM_HandleTypeDef htim2;

/* Handler for TIM2*/
UART_HandleTypeDef huart2;

/* user LED input*/
char in_led[3];

/* duty cycle provided by user in string buffer form */
char duty_cycle_str[3*sizeof(char) + 1] = "100";

/* pulse width to modify timer modules' PWM output*/
uint32_t pulse_width;



int str_to_int(char* str)
{
	// Initialize result
	int res = 0;

	// Iterate through all characters
	// of input string and update result
	// take ASCII character of corosponding digit and
	// subtract the code from '0' to get numerical
	// value and multiply res by 10 to shuffle
	// digits left to update running total
	for (int i = 0; str[i] != '\0'; ++i)
		res = res * 10 + str[i] - '0';

	return res;
}

unsigned int strlength(char *p)
{
	unsigned int count = 0;

	while (*p!='\0') {
		count++;
		p++;
	}

	return count;
}

void print_message(char msg[])
{
	HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlength(msg), 100);
}

uint32_t calculate_pulse_width(uint8_t duty_percent)
{
	float pulse_width = (duty_percent * 200) / ((float) 100);
	return pulse_width;
}

uint8_t get_duty_cycle_int()
{
	char *duty_cycle_str_l = &duty_cycle_str[0];
	return str_to_int(duty_cycle_str_l);
}

void print_duty_cycle_prompt()
{
	print_message("Please enter LED power in %.\r\n");
	print_message("The format is as follows:\r\n");
	print_message("		001		-		1%\r\n");
	print_message("		010		-		10%\r\n");
	print_message("		100		-		100%\r\n");
	print_message("Enter: \r\n\r\n");
}

int receive_duty_cycle()
{
	char *duty_cycle_str_l = &duty_cycle_str[0];

	HAL_UART_Receive(&huart2,(uint8_t *) duty_cycle_str_l,
			strlength(duty_cycle_str_l), HAL_MAX_DELAY);

	uint32_t led_power = get_duty_cycle_int();
	if (led_power >= 0 && led_power <= 100) {
		pulse_width = calculate_pulse_width(led_power);
		return 1;
	} else {
		print_message("Invalid percentage has been entered. Try again:\r\n");
		return 0;
	}

}

void print_led_choice()
{
	char select1[] = "You have selected";
	print_message(select1);
	if (in_led[0] == '1') {
		print_message(": onboard LED.\r\n");
	} else if (in_led[0] == '2') {
		print_message(": external LED.\r\n");
	} else{
		print_message(" an invalid LED. Try again...\r\n");
	}
}

uint8_t check_led_input()
{
	if (in_led[0] == '1' || in_led[0] == '2') {
		print_led_choice();
		return 1;
	} else {
		print_led_choice();
		return 0;
	}
}

uint8_t receive_led_input()
{
	if (HAL_UART_Receive(&huart2, (uint8_t *) in_led, sizeof(char), HAL_MAX_DELAY) == HAL_OK) {
		return check_led_input();
	} else {
		return 0;
	}
}

void print_led_prompt()
{
	char request_led[] = "Please Enter 1 for onboard LED or 2 for external LED:\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) request_led, sizeof(request_led), 100);
}

void credential_print()
{
	char credentials[] = "Assignment 3 - <Mark> <Bilginer>\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) credentials, sizeof(credentials), 100);

}

void carriage_newline_print()
{
	char in_end[] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) in_end, sizeof(in_end), 100);
}

void print_led_percentage()
{
	print_message("Setting LED to ");
	print_message(duty_cycle_str);
	print_message("% power...\r\n");
}

void dim_chosen_led()
{
	if (in_led[0] == '2') {
		adjust_tim1_pulse_width(pulse_width);
	} else if (in_led[0] == '1') {
		adjust_tim2_pulse_width(pulse_width);
	} else {
		//do_nothing
	}
}

void config_system_clock(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure LSE Drive Capability
	*/
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	*/
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	*/
	HAL_RCCEx_EnableMSIPLLMode();
}

void adjust_tim1_pulse_width(uint32_t pulse_width)
{
	htim1.Instance->CCR2 = pulse_width;
}

void adjust_tim2_pulse_width(uint32_t pulse_width)
{
	htim2.Instance->CCR2 = pulse_width;
}

void init_tim1(uint32_t prescaler, uint32_t period, uint32_t pulse)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = prescaler;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = period;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);
}

void init_tim2(uint32_t prescaler, uint32_t period, uint32_t pulse)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = prescaler;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = period;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim2);
}

void init_usart2_uart(uint32_t baudrate)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = baudrate;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

void init_gpio(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

void complement_tim_pwm_start()
{
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void tim_pwm_start()
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void init_hal()
{
	HAL_Init();
}

void Error_Handler(void)
{
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {

	}
}
