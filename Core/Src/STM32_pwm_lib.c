/*
 * STM32_pwm_lib.c
 *
 *  Created on: May 28, 2021
 *      Author: mark
 */


#include "STM32_pwm_lib.h"


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;


static int which_led(char n_led);
static void led_percentage(size_t sz, uint8_t num[]);

char in_led[3];
char duty_cycle_str[3*sizeof(char) + 1] = "100";
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

    // return result.
    return res;
}

unsigned int strlength(char *p)
{
	unsigned int count = 0;

	while(*p!='\0')
	{
		count++;
		p++;
	}

	return count;
}
void print_message(char msg[]){
	  HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlength(msg), 100);
}

//uint8_t convert_digits_to_number(){
//	uint8_t number = 10*duty_cycle[1] + duty_cycle[0];
//	return number;
//}

uint32_t calculate_pulse_width(uint8_t duty_percent){

	float pulse_width = (duty_percent * 200) / ((float) 100);

	return pulse_width;
}

uint8_t get_duty_cycle_int(){
	char *duty_cycle_str_l = &duty_cycle_str[0];
	return str_to_int(duty_cycle_str_l);
}

void print_duty_cycle_prompt(){
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
	if(led_power >= 0 && led_power <= 100){
		pulse_width = calculate_pulse_width(led_power);
		return 1;
	}
	else{
		print_message("Invalid percentage has been entered. Try again:\r\n");
		return 0;
	}

}

uint8_t check_led_input(){
	if(in_led[0] == '1' || in_led[0] == '2'){
		return 1;
	} else {
		print_message("Invalid LED has been chosen. Try again...\r\n");
		return 0;
	}
}

uint8_t receive_led_input(){

	if(HAL_UART_Receive(&huart2, (uint8_t *) in_led, sizeof(char), HAL_MAX_DELAY) == HAL_OK){
		return check_led_input();
	} else {
		return 0;
	}
}

void print_led_prompt(){
	  char request_led[] = "Please Enter 1 for onboard LED or 2 for external LED:\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *) request_led, sizeof(request_led), 100);
}

void credential_print(){

	char credentials[] = "Assignment 3 - <Mark> <Bilginer>\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) credentials, sizeof(credentials), 100);

}

void carriage_newline_print(){

	char in_end[] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) in_end, sizeof(in_end), 100);
}

void led_percentage_print(){
	led_percentage(strlength(duty_cycle_str), duty_cycle_str);
}


static void led_percentage(size_t sz, uint8_t num[]){

	char select1[] = "Setting LED to ";
	HAL_UART_Transmit(&huart2, (uint8_t *)select1, sizeof(select1), 10);

	HAL_UART_Transmit(&huart2, (uint8_t*)num, sz, 100);

	char select2[] = "% power...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *)select2, sizeof(select2), 1000);
}

void dim_chosen_led(){

	if(chosen_led() == '2'){
		adjust_tim1_pulse_width(pulse_width);
	}
	else if(chosen_led() == '1'){
		adjust_tim2_pulse_width(pulse_width);
	}
	else{
		//do_nothing
	}
}

char chosen_led(){
	return in_led[0];
}

int which_led_print(){
	return which_led(in_led[0]);
}

static int which_led(char n_led){

	char select1[] = "You have selected ";

	if(n_led == '1'){
		HAL_UART_Transmit(&huart2, (uint8_t *)select1, sizeof(select1), 100);
		char onboard_led[] = "onboard LED.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)onboard_led, sizeof(onboard_led), 100);
	}
	else if(n_led == '2'){
		HAL_UART_Transmit(&huart2, (uint8_t *)select1, sizeof(select1), 100);
		char external_led[] = "external LED.\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)external_led, sizeof(external_led), 100);
	}
	else{
		char invalid_num[] = "You have entered an invalid number. Try again...\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)invalid_num, sizeof(invalid_num), 100);

		return 0;
	}
	return 1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void adjust_tim1_pulse_width(uint32_t pulse_width){
	htim1.Instance->CCR2 = pulse_width;
}

void adjust_tim2_pulse_width(uint32_t pulse_width){
	htim2.Instance->CCR2 = pulse_width;
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void init_tim1(uint32_t prescaler, uint32_t period, uint32_t pulse)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = prescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = period;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void init_tim2(uint32_t prescaler, uint32_t period, uint32_t pulse)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = prescaler;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void init_usart2_uart(uint32_t baudrate)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void init_gpio(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

void complement_tim_pwm_start(){
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

void tim_pwm_start(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void init_hal(){
	HAL_Init();
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
