/* this library uses TIM16 as Output Compare for triac control*/

#include "my_output_compare.h" 
#include "my_ac_control.h"

bool_t interruptOCFlag=FALSE; //if there is interrupt -> TRUE

void HAL_TIM_OC_CaptureCallback(TIM_HandleTypeDef htim16)
{
		interruptOCFlag= TRUE; // Si hubo interrupcion de TIM16 activo flag
	
}	

void OCInterrupt(void)
{
	if(interruptOCFlag==TRUE)
	{
		HAL_GPIO_WritePin(GPIOA,LED2_Pin,GPIO_PIN_SET);
		for(int i=0;i<10;i++)  
		{
			__asm("nop");
		}
		HAL_GPIO_WritePin(GPIOA,LED2_Pin,GPIO_PIN_RESET);
		interruptOCFlag=FALSE;
	}
}


void shootChange(TIM_HandleTypeDef *tim,uint8_t shootVal)// 
{
	tick_t shootOut=0,maxTickValue=0;
	
	if(	shootVal>SHOOTMAX)
		shootVal=SHOOTMAX;
	
	maxTickValue=getDifference(); // get ticks between ac interputs
	maxTickValue/=SHOOTMAX;
	
	shootOut=maxTickValue*shootVal;
	
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, shootOut);//update pwm value
}

void resetOCCouenter(TIM_HandleTypeDef *tim)
{
		__HAL_TIM_SetCounter(tim,0);	
}




