#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "RTE_Components.h"             // Component selection
#include "Segments.h" 
uint16_t segArray[20] = {SEGMENT_ZERO, SEGMENT_ONE, SEGMENT_TWO, SEGMENT_THREE, 
												SEGMENT_FOUR, SEGMENT_FIVE, SEGMENT_SIX, SEGMENT_SEVEN, 
												SEGMENT_EIGHT, SEGMENT_NINE, SEGMENT_ZERO_D, SEGMENT_ONE_D, 
												SEGMENT_TWO_D, SEGMENT_THREE_D, SEGMENT_FOUR_D, 
												SEGMENT_FIVE_D, SEGMENT_SIX_D, SEGMENT_SEVEN_D, 
												SEGMENT_EIGHT_D, SEGMENT_NINE_D};

uint16_t getSegments(int digit, float number)
{
	int temp = 0;
	if(number > 100)
	{		
		switch(digit)
		{
			case 0:
				temp = (int)(number/100);
				break;
			case 1:
				temp = ((int)(number/10))%10;
				break;
			case 2:
				 temp = ((int)number)%10;
				break;
		}
		return segArray[temp];
		// 100
		// 10
		// 1
	}
	else if(number > 10)
	{
		if(digit == 1)
			temp += 10;
		switch(digit)
		{
			case 0:
				temp += ((int)(number/10))%10;
				break;
			case 1:
				temp += ((int)(number))%10;
				break;
			case 2:
				 temp += ((int)(number*10))%10;
				break;
		}
		return segArray[temp];
		// 10
		// 1
		// 0.1
	}
	else
	{
		if(digit == 0)
			temp += 10;
		switch(digit)
		{
			case 0:
				temp += ((int)number);
				break;
			case 1:
				temp += ((int)(number*10)%10);
				break;
			case 2:
				 temp += ((int)(number*100)%10);
				break;
		}
		return segArray[temp];
		// 1
		// 0.1
		// 0.01
	}
}

void getSegmentsTemp(int digit)
{
	
}

//Sets ad reset each select pin as well as the pins for each segement
//Also insert delay such that digits appear as constant to the eye
void updateDisplay(int display, float number)
{			
	if(display == 0)
	{
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FOURTH,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,0xFFFF,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FIRST,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,getSegments(display, number),GPIO_PIN_SET);
	}
	else if(display == 1)
	{
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FIRST,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,0xFFFF,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_SECOND,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,getSegments(display, number),GPIO_PIN_SET);
	}
	else if(display == 2)
	{
		HAL_GPIO_WritePin(GPIOA,SEGMENT_SECOND,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,0xFFFF,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_THIRD,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,getSegments(display, number),GPIO_PIN_SET);
	}
	else if(display == 3)
	{
		HAL_GPIO_WritePin(GPIOA,SEGMENT_THIRD,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,0xFFFF,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,SEGMENT_FOURTH,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,SEGMENT_DEGREE,GPIO_PIN_SET);
	}
}