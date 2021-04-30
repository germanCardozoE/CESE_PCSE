/* this library use TIM3 for PWM control and */

#include "my_pwm.h" 

void duttyChange(TIM_HandleTypeDef *tim,uint8_t duttyVal)// pwm timer, value
{
	tick_t period=0,duttyOut=0;
	
	if(	duttyVal>PWMMAX)
		duttyVal=PWMMAX;

	period=tim->Init.Period;// = 400;
	period/=PWMMAX;
	duttyOut=period*duttyVal;
	
	__HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, duttyOut);//update pwm value
}

/*
tick_t getPeriod(TIM_HandleTypeDef *tim)
{
	tick_t period;
	period=tim->Init.Period;
	return period;
}


void setPeriod(TIM_HandleTypeDef *tim,tick_t newVal)
{
	tim->Init.Period=newVal;
}
*/

