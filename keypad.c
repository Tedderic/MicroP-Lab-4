#include <stdio.h>
#include "stm32f4xx_hal.h"               // Device header
#include "keypad.h"
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"

void Thread_keypad (void const *argument);                 // thread function
osThreadId tid_Thread_keypad;                              // thread id
osThreadDef(Thread_keypad, osPriorityNormal, 1, 0);

int colDet = 0;
int rowDet = 0;
int value;
int keypad_value;
extern osMutexId mutex;

#define COLUMN_PIN_SET (uint16_t)(GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6)
#define ROW_PIN_SET 	 (uint16_t)(GPIO_PIN_9 | GPIO_PIN_2 | GPIO_PIN_4| GPIO_PIN_5)

// should be 1 and 6
int read_keypad() {
	
	read_keypad_col();
	if (colDet != 0) {
		read_keypad_row();
		value = interpret_keypad();
		colDet = 0;
		rowDet = 0;
		return value;
	}
	
	return -1;
}

void read_keypad_col() {
	initGPIOT(GPIOB, COLUMN_PIN_SET, input);//Used for the keypad
	initGPIOT(GPIOB, ROW_PIN_SET, output);
	
	// set all rows to 0
	HAL_GPIO_WritePin(GPIOB,ROW_PIN_SET,GPIO_PIN_RESET);
	
	// loop through all columns, see which is right!!!
	//while (!colDet) {
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) colDet = 1;
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) colDet = 2;
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) colDet = 3;
	//}
}

void read_keypad_row() { 
	
	// initialize col = output, row = input
	initGPIOT(GPIOB, COLUMN_PIN_SET, output);//Used for the keypad
	initGPIOT(GPIOB, ROW_PIN_SET, input);
	
	// set all cols to 0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	
	// loop through all rows, see which is right!!!
	while (!rowDet) {
		if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) rowDet = 1;
		else if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) rowDet = 3;
		else if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) rowDet = 4;
		else if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)) rowDet = 2;
	}
	
	// reset initialization
	initGPIOT(GPIOB, COLUMN_PIN_SET, input);//Used for the keypad
	initGPIOT(GPIOB, ROW_PIN_SET, output);
}

void initGPIOT(GPIO_TypeDef* GPIOx, uint16_t pins, enum IOKey inOut)
{	
	GPIO_InitTypeDef GPIOInit;
	GPIOInit.Alternate = 0;	
	GPIOInit.Pin = pins;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInit.Pull = GPIO_PULLUP;
	if(inOut == output)
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	else
		GPIOInit.Mode = GPIO_MODE_INPUT;

	//Enable GPIO	
	HAL_GPIO_Init(GPIOx, &GPIOInit);
}

int interpret_keypad()
{
	switch(rowDet) {
		case 1 :
			switch(colDet) {
				case 1: return 1;
				case 2: return 2;
				case 3: return 3;
				default: return -1;
			}
		case 2 :
			switch(colDet) {
				case 1: return 4;
				case 2: return 5;
				case 3: return 6;
				default: return -1;
			}
		case 3 :
			switch(colDet) {
				case 1: return 7;
				case 2: return 8;
				case 3: return 9;
				default: return -1;
			}
		case 4 :
			switch(colDet) {
				case 1: return 100;
				case 2: return 0;
				case 3: return 1000;
				default: return -1;
			}
		default: return -1;
	}
}



/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_keypad (void) {

  tid_Thread_keypad = osThreadCreate(osThread(Thread_keypad ), NULL); // Start LED_Thread
  if (!tid_Thread_keypad) return(-1); 
  return(0);
}

 /*----------------------------------------------------------------------------
*      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
	void Thread_keypad (void const *argument) {
		while(1){
				osMutexWait(mutex, 100);
				keypad_value = read_keypad();
				osMutexRelease(mutex);
				osDelay(1000);
			}
	}
/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
void initialize_Keypad (void){
			__HAL_RCC_GPIOB_CLK_ENABLE();
			initGPIOT(GPIOB, 0xFFF7, output);
}
	
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/