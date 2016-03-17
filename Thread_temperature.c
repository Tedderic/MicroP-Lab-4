	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"
#include "Segments.h" 

void Thread_Temp (void const *argument);                 // thread function
float tempConv(uint32_t voltage);
osThreadId tid_Thread_Temp;                              // thread id
osThreadDef(Thread_Temp, osPriorityNormal, 1, 0);
ADC_HandleTypeDef ADCHandleinit;
ADC_InitTypeDef ADCInit;
ADC_ChannelConfTypeDef channelConfig;

float rawValue = 0;
float convertedValue = 0;
int digit = 0;
int test =100;
extern TIM_HandleTypeDef timerHandle;


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
		
		HAL_TIM_Base_Init(&timerHandle);
		HAL_TIM_Base_Start_IT(&timerHandle);
		while(1){
						HAL_Delay(500);
						HAL_ADC_Start(&ADCHandleinit);
						HAL_ADC_PollForConversion(&ADCHandleinit, 10);
						rawValue = HAL_ADC_GetValue(&ADCHandleinit);
						convertedValue = tempConv(rawValue);
			
						__HAL_ADC_CLEAR_FLAG(&ADCHandleinit,ADC_FLAG_EOC);
			}
	}
/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
	void initialize_Temp (void)
	{		
		//Enable TIM3 interrupt
		HAL_NVIC_EnableIRQ(TIM3_IRQn);
		//Sets priority to maximum 
		HAL_NVIC_SetPriority(TIM3_IRQn,1, 1);
		
		timerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		timerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
		timerHandle.Init.Period = 4501;
		timerHandle.Init.Prescaler = 5;
		
		timerHandle.Instance = TIM3;		
		ADCInit.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		ADCInit.ContinuousConvMode = DISABLE;
		ADCInit.DataAlign = ADC_DATAALIGN_RIGHT;
		ADCInit.DiscontinuousConvMode = DISABLE;
		ADCInit.DMAContinuousRequests = DISABLE;
		ADCInit.EOCSelection = ADC_EOC_SINGLE_CONV;
		ADCInit.ExternalTrigConv = ADC_SOFTWARE_START;
		ADCInit.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		ADCInit.NbrOfConversion= 1;
		ADCInit.NbrOfDiscConversion = 0;
		ADCInit.Resolution = ADC_RESOLUTION_12B;
		ADCInit.ScanConvMode = DISABLE;

		ADCHandleinit.ErrorCode = HAL_ADC_ERROR_NONE;
		
		ADCHandleinit.Init.ClockPrescaler = ADCInit.ClockPrescaler;
		ADCHandleinit.Init.ContinuousConvMode = ADCInit.ContinuousConvMode;
		ADCHandleinit.Init.DataAlign = ADCInit.DataAlign;
		ADCHandleinit.Init.DiscontinuousConvMode = ADCInit.DiscontinuousConvMode;
		ADCHandleinit.Init.DMAContinuousRequests = ADCInit.DMAContinuousRequests;
		ADCHandleinit.Init.EOCSelection = ADCInit.EOCSelection;
		ADCHandleinit.Init.ExternalTrigConv = ADCInit.ExternalTrigConv;
		ADCHandleinit.Init.ExternalTrigConvEdge = ADCInit.ExternalTrigConvEdge;
		ADCHandleinit.Init.NbrOfConversion = ADCInit.NbrOfConversion;
		ADCHandleinit.Init.NbrOfDiscConversion = ADCInit.NbrOfDiscConversion;
		ADCHandleinit.Init.Resolution = ADCInit.Resolution;
		ADCHandleinit.Init.ScanConvMode = ADCInit.ScanConvMode;
		
		ADCHandleinit.Instance = ADC1;
		ADCHandleinit.Lock = HAL_UNLOCKED;
		ADCHandleinit.NbrOfCurrentConversionRank = 1;
		
		HAL_ADC_Init(&ADCHandleinit);
		__HAL_RCC_ADC1_CLK_ENABLE();		
		
		channelConfig.Channel = ADC_CHANNEL_16;
		channelConfig.Offset = 0;
		channelConfig.Rank = 1;
		channelConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;

		HAL_ADC_ConfigChannel(&ADCHandleinit, &channelConfig);
}
	
/*Temperature Conversion*/
float tempConv(uint32_t voltage)
{
	float temperature = 0;
	
	/* 0.76 = 2.5(25) + constant */
	float slope = 0.0025;
	float roomTemp = 25;// 25 degree
	float roomVoltage = 0.76;// Voltage at 25 degree in millivolts

	temperature = ((float)voltage*3000)/(4095*1000);
	temperature = ((temperature - roomVoltage)/slope) + roomTemp;
	return temperature;
}

void TIM3_IRQHandler(void)
{
	test++;
	updateDisplay(digit++%4, convertedValue);
	HAL_TIM_IRQHandler(&timerHandle);	
}
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/
