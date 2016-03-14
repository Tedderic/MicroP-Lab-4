	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"

void Thread_Temp (void const *argument);                 // thread function
osThreadId tid_Thread_Temp;                              // thread id
osThreadDef(Thread_Temp, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_Temp (void) {

  tid_Thread_Temp = osThreadCreate(osThread(Thread_Temp ), NULL); // Start LED_Thread
  if (!tid_Thread_Temp) return(-1); 
  return(0);
}

 /*----------------------------------------------------------------------------
*      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
	void Thread_Temp (void const *argument) {
		while(1){
				osDelay(1000);
				HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			}
	}
/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
	void initialize_Temp (void){
	
	__HAL_RCC_ADC1_CLK_ENABLE();	
}
	
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/