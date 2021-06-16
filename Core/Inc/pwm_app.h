/*
 * pwm_app.h
 *
 *  Created on: 28 May 2021
 *      Author: mark
 */

#ifndef INC_PWM_APP_H_
#define INC_PWM_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "STM32_pwm_lib.h"

/* Exported functions prototypes ---------------------------------------------*/
void start_application();
void start_pwm();
void setup_micro();

#ifdef __cplusplus
}
#endif

#endif /* INC_PWM_APP_H_ */
