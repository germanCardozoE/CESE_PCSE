/* this library uses TIM14 as Input Capture for zero detection*/

#include "my_ac_control.h" 
//tick_t zero;
 bool_t interruptZeroFlag=FALSE; //if there is interrupt -> TRUE
 tick_t frecuency; 
 tick_t clkfrec;
 tick_t Difference ; // ticks between interrupts


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
		interruptZeroFlag= TRUE; // TIM14 interrupt
}	

bool_t getZeroInterruptFlag(void)
{
	return interruptZeroFlag;
}

tick_t getDifference(void)
{
	return Difference;
}

void clrZeroFlag(void)
{
		interruptZeroFlag=FALSE;
}

// no blocking interrupt attention
void Zero_Interrupt(TIM_HandleTypeDef *htim)
{
		static tick_t IC_Value1 ;
		static tick_t IC_Value2,frecuencia;

		static bool_t Is_First_Captured;  // 0- not captured, 1- captured
		
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
		{
				if (Is_First_Captured==TRUE)  // is the first value captured ? 
				{
						IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture the first value
						Is_First_Captured =FALSE;  // set the first value captured as true
				}
				else if (Is_First_Captured)  // if the first is captured
				{
						IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // capture second value

						if (IC_Value2 > IC_Value1)  
						{
								Difference = IC_Value2-IC_Value1;   // calculate the difference
						}
						else if (IC_Value2 < IC_Value1)
						{
								Difference = ((0xffff-IC_Value1)+IC_Value2) +1;
						}
						else
						{
								Error_Handler();
						}
						clkfrec=HAL_RCC_GetPCLK1Freq()/(htim->Init.Prescaler);
						frecuency = (clkfrec/Difference);  // calculate frequency
						frecuencia=frecuency;
						Is_First_Captured = TRUE;  // reset the first captured
				}
				
		}
}

uint16_t getACFrecuency()
{
	return frecuency;
}

uint16_t getClockFrecuency(TIM_HandleTypeDef *htim)
{
	clkfrec=HAL_RCC_GetPCLK1Freq()/(htim->Init.Prescaler);
	return clkfrec;
}






