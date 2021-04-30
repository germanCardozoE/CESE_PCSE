/* this library uses TIM16 as Output Compare for triac control*/

#include "data_types.h"



void HAL_TIM_OC_CaptureCallback(TIM_HandleTypeDef htim16);

void OCInterrupt(void);


void shootChange(TIM_HandleTypeDef *,uint8_t);

void resetOCCouenter(TIM_HandleTypeDef *);
	








