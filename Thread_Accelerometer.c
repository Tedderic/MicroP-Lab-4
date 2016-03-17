	
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "stm32f4xx_hal.h"
#include "Segments.h" 
#include "lis3dsh.h"
#include "math.h"

void Thread_Accel (void const *argument);                 // thread function
float tempConv(uint32_t voltage);
osThreadId tid_Thread_Accel;                              // thread id
osThreadDef(Thread_Accel, osPriorityNormal, 1, 0);

int digit = 0;
float detectedValue = 0;
const double PI = 3.1415926535;
LIS3DSH_InitTypeDef ACCEL_INIT;
LIS3DSH_DRYInterruptConfigTypeDef ACCEL_INT_INIT;
TIM_HandleTypeDef timerHandle;
float calibrationMatrix[4][3];
float calibratedAcc[3];

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
int start_Thread_Accel (void) {

  tid_Thread_Accel = osThreadCreate(osThread(Thread_Accel ), NULL); // Start LED_Thread
  if (!tid_Thread_Accel) return(-1); 
  return(0);
}

 /*----------------------------------------------------------------------------
*      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/
	void Thread_Accel (void const *argument) {
		HAL_TIM_Base_Init(&timerHandle);
		HAL_TIM_Base_Start_IT(&timerHandle);
		while(1){
						
			}
	}
/*----------------------------------------------------------------------------
 *      Initialize the GPIO associated with the LED
 *---------------------------------------------------------------------------*/
void initialize_Accel (void)
{				
		timerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		timerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
		timerHandle.Init.Period = 4501;
		timerHandle.Init.Prescaler = 5;
		
		timerHandle.Instance = TIM2;	
		//Initialize accelerometer structure
		ACCEL_INIT.AA_Filter_BW = LIS3DSH_AA_BW_50;
		ACCEL_INIT.Axes_Enable = LIS3DSH_XYZ_ENABLE;
		ACCEL_INIT.Continous_Update = LIS3DSH_ContinousUpdate_Disabled;
		ACCEL_INIT.Full_Scale =LIS3DSH_FULLSCALE_2;
		ACCEL_INIT.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_25;
		
		//Initialize accelerometer interrupts structure
		ACCEL_INT_INIT.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;
		ACCEL_INT_INIT.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;
		ACCEL_INT_INIT.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;
		//Initialize Accelerometer
		LIS3DSH_Init(&ACCEL_INIT);
		//Initialize interrupts
		LIS3DSH_DataReadyInterruptConfig(&ACCEL_INT_INIT);
		//Enable external interrupt line 0
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		//Sets prioity of the external interrupt
		HAL_NVIC_SetPriority(EXTI0_IRQn,3, 1);
		//Enable TIM3 interrupt
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
		//Sets priority to maximum 
		HAL_NVIC_SetPriority(TIM2_IRQn,2, 1);
}

void storeCalibrationMatrix()
{
	calibrationMatrix[0][0] = 0.000971848026340;	calibrationMatrix[0][1] = -0.000019145822271;	calibrationMatrix[0][2] = -0.000006527715576;
	calibrationMatrix[1][0] = -0.000020230289300;	calibrationMatrix[1][1] = 0.001000536201152;	calibrationMatrix[1][2] = -0.000011853914413;
	calibrationMatrix[2][0] = -0.000010679237958;	calibrationMatrix[2][1] = 0.000014078793958;	calibrationMatrix[2][2] = 0.000998443347062;
	calibrationMatrix[3][0] = -0.000630972996122;	calibrationMatrix[3][1] = -0.011612009697235;	calibrationMatrix[3][2] = -0.019155796902639;
}

void applyMatrix(float Ax, float Ay, float Az)
{
	calibratedAcc[0] = Ax*calibrationMatrix[0][0] + Ay*calibrationMatrix[1][0] + Az*calibrationMatrix[2][0] + calibrationMatrix[3][0];
	calibratedAcc[1] = Ax*calibrationMatrix[0][1] + Ay*calibrationMatrix[1][1] + Az*calibrationMatrix[2][1] + calibrationMatrix[3][1];
	calibratedAcc[2] = Ax*calibrationMatrix[0][2] + Ay*calibrationMatrix[1][2] + Az*calibrationMatrix[2][2] + calibrationMatrix[3][2];
}

//Compute angles
void computeAngles(double Ax, double Ay,double Az, float* angles)
{
	double pitchInRad = atan(Ax / sqrt(Ay*(Ay) + Az*(Az)));
	double rollInRad = atan(Ay / sqrt(Ax*(Ax) + Az*(Az)));
	//Convert to degrees
	pitchInRad = (pitchInRad * 360 )/ (2 * PI);
	rollInRad = (rollInRad * 360 )/ (2 * PI);
	//Return results
	angles[0] = pitchInRad;
	angles[1] = rollInRad;
}

void TIM2_IRQHandler(void)
{
	updateDisplay(digit++%4, detectedValue);
	HAL_TIM_IRQHandler(&timerHandle);	
}
/*----------------------------------------------------------------------------
 *      
 *---------------------------------------------------------------------------*/
