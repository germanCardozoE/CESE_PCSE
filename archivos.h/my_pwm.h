/* this library use TIM3 for PWM control */
#include "main.h"
#include "data_types.h"


void duttyChange(TIM_HandleTypeDef *,uint8_t);
void setPeriod(TIM_HandleTypeDef *,tick_t);
tick_t getPeriod(TIM_HandleTypeDef *);

