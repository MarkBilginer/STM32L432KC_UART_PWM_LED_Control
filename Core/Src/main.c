/** @file main.c
 *  @brief Application run is contained in this file.
 *
 *  This file contains the applications' main() function.
 *
 *  This is the entry point for the application.
 *  Application name is STM32_UART_PWM_LED_Control.
 *  What the Application does? Lets user choose
 *  between on-board and external LED. Then asks
 *  the user the brightness of the LED.
 *
 *  @author Mark Bilginer (GitHub: MarkBilginer)
 *  @bug no known bugs.
 */

/* -- Includes -- */
#include "pwm_app.h"


/** @brief The application entry point.
  * @return Should not return.
*/
int main(void)
{
	setup_micro();
	start_pwm();

	while (1) {
		start_application();
	}
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

