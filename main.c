/*******************************************************************************
  * @file    main.c
  * @author  Ashraf Suyyagh
	* @version V1.2.0
  * @date    17-January-2016
  * @brief   This file demonstrates flasing one LED at an interval of one second
	*          RTX based using CMSIS-RTOS 
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "RTE_Components.h"             // Component selection
#include "Segments.h" 

extern void initializeLED_IO			(void);
extern void initialize_Temp 			(void);
extern void initialize_Accel			(void);
extern void initialize_Keypad			(void);

extern void start_Thread_LED			(void);
extern void start_Thread_Temp     (void);
extern void start_Thread_Accel    (void);
extern void start_Thread_keypad		(void);

extern void Thread_LED		(void const *argument);
extern void Thread_Temp 	(void const *argument);
extern void Thread_Accel 	(void const *argument);
extern void Thread_keypad (void const *argument);

extern osThreadId tid_Thread_LED;
extern int keypad_value;
void initGPIO(GPIO_TypeDef* GPIOx, uint16_t pins, uint16_t input);

void thread1 (void);
void thread2 (void);
int valueType = 1;
float convertedValue[3];
osMutexId mutex;
TIM_HandleTypeDef timerHandle;
osThreadId thrdID1,thrdID2;
int digit = 0;
/**
	These lines are mandatory to make CMSIS-RTOS RTX work with te new Cube HAL
*/
#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/**
  * System Clock Configuration
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * Main function
  */
int main (void) {
	//osMutexDef(mutex);
  osKernelInitialize();                     /* initialize CMSIS-RTOS          */
  HAL_Init();                               /* Initialize the HAL Library     */
  SystemClock_Config();                     /* Configure the System Clock     */

	/* User codes goes here*/
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
		//Enable TIM3 interrupt
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
		//Sets priority to maximum 
		HAL_NVIC_SetPriority(TIM3_IRQn,1, 1);
		
		timerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		timerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
		timerHandle.Init.Period = 4501;
		timerHandle.Init.Prescaler = 5;
		
		timerHandle.Instance = TIM3;		
		HAL_TIM_Base_Init(&timerHandle);
		HAL_TIM_Base_Start_IT(&timerHandle);
	initGPIO(GPIOA, 0x9FFF, 0);//Used as a select signal
	initGPIO(GPIOC, 0xFFFF, 0);//Used to control segments
  initializeLED_IO();                       /* Initialize LED GPIO Buttons    */
	initialize_Temp();
	initialize_Accel();
	initialize_Keypad();
  
	start_Thread_LED();                       /* Create LED thread              */
	start_Thread_Temp();
	start_Thread_Accel();
	start_Thread_keypad();
	/* User codes ends here*/
  
	osKernelStart();                          /* start thread execution         */
}

//Initialize GPIO
void initGPIO(GPIO_TypeDef* GPIOx, uint16_t pins, uint16_t input)
{	
	GPIO_InitTypeDef GPIOInit;
	GPIOInit.Alternate = 0;	
	GPIOInit.Pin = pins;
	GPIOInit.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIOInit.Pull = GPIO_PULLUP;
	if(!input)
		GPIOInit.Mode = GPIO_MODE_OUTPUT_PP;
	else
		GPIOInit.Mode = GPIO_MODE_INPUT;

	//Enable GPIO	
	HAL_GPIO_Init(GPIOx, &GPIOInit);
}

void TIM3_IRQHandler(void)
{
	/*******Mutex*********/
	osMutexWait(mutex, 100);
	
	if(keypad_value<4 && keypad_value>0)
		valueType = keypad_value;
	
	osMutexRelease(mutex);
	/*******Mutex*********/
	updateDisplay(digit++%4, convertedValue[valueType-1]);
	HAL_TIM_IRQHandler(&timerHandle);	
}
