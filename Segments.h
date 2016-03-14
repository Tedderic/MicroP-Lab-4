#ifndef SEVENSEGMENT_H
#define SEVENSEGMENT_H

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"
//	     A0	
//	  	____
//F5	 |    |  B1
//	 	 |_G6_|
//E4	 |    |  C2
//	 	 |____|
//       D3
//Select bit 1 - 2 - 3 - 4

//Constants to specify the segements for each number to display
#define SEGMENT_NINE 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6)
#define SEGMENT_EIGHT 	(uint16_t)(GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6)
#define SEGMENT_SEVEN 	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2)
#define SEGMENT_SIX 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_3)
#define SEGMENT_FIVE 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_3)
#define SEGMENT_FOUR 		(uint16_t)(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6)
#define SEGMENT_THREE 	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6)
#define SEGMENT_TWO 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6)
#define SEGMENT_ONE 		(uint16_t)(GPIO_PIN_1|GPIO_PIN_2)
#define SEGMENT_ZERO 		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5)
#define SEGMENT_NINE_D	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)
#define SEGMENT_EIGHT_D	(uint16_t)(GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7)
#define SEGMENT_SEVEN_D	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7)
#define SEGMENT_SIX_D		(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_7)
#define SEGMENT_FIVE_D	(uint16_t)(GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7)
#define SEGMENT_FOUR_D	(uint16_t)(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7)
#define SEGMENT_THREE_D	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7)
#define SEGMENT_TWO_D		(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7)
#define SEGMENT_ONE_D		(uint16_t)(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7)
#define SEGMENT_ZERO_D	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7)
#define SEGMENT_DEGREE	(uint16_t)(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_5)
#define SEGMENT_DECIMAL	(unit16_t)(GPIO_PIN_7)
#define SEGMENT_ALL			(unit16_t)(GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6)

//Constant to define which digit to display
#define SEGMENT_FIRST		(uint16_t) GPIO_PIN_0
#define SEGMENT_SECOND 	(uint16_t) GPIO_PIN_1
#define SEGMENT_THIRD		(uint16_t) GPIO_PIN_2
#define SEGMENT_FOURTH	(uint16_t) GPIO_PIN_3

void updateDisplay(int display);

#endif
