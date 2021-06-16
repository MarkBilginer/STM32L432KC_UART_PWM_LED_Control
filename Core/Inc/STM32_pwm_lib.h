/*
 * STM32_pwm_lib.h
 *
 *  Created on: May 28, 2021
 *      Author: mark
 */

#ifndef INC_STM32_PWM_LIB_H_
#define INC_STM32_PWM_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);
void init_hal(void);
void config_system_clock(void);
void init_gpio(void);
void init_usart2_uart(uint32_t baudrate);
void init_tim1(uint32_t prescaler, uint32_t period, uint32_t pulse);
void init_tim2(uint32_t prescaler, uint32_t period, uint32_t pulse);
int which_led_print();
void led_percentage_print();
void carriage_newline_print();
unsigned int my_strlen(const char *s);
void credential_print();
void print_led_prompt();
void print_duty_cycle_prompt();
void complement_tim_pwm_start();
void tim_pwm_start();
uint8_t receive_led_input();
int receive_duty_cycle();
void adjust_tim2_pulse_width(uint32_t pulse_width);
void adjust_tim1_pulse_width(uint32_t pulse_width);
uint32_t calculate_duty_cycle(uint32_t duty_cycle_percent);
void dim_chosen_led(void);
char chosen_led();
uint8_t convert_digits_to_number();

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32_PWM_LIB_H_ */
