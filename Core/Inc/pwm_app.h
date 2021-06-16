/** @file pwm_app.h
 *  @brief Application layer helper and wrapper function declarations.
 *  Aims to make the code more readable. Application layer,
 *  library layer and driver layer concept was in mind.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug No known bugs.
 */

#ifndef INC_PWM_APP_H_
#define INC_PWM_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* -- Includes -- */

/* -- Library abstraction layer -- */
#include "STM32_pwm_lib.h"

/* -- Exported functions prototypes -- */

/** @brief Setup necessary functions of MCU.
 *
 *  Wrapper function.
 *
 *  @param None
 *  @return None
 */
void setup_micro();

/* -- Exported functions prototypes -- */

/** @brief Enables and starts PWM.
 *
 *  Wrapper function.
 *
 *  @param None
 *  @return None
 */
void start_pwm();

/** @brief The application logic flow is here.
 *  @param None
 *  @return None
 */
void start_application();

#ifdef __cplusplus
}
#endif

#endif /* INC_PWM_APP_H_ */
