/* this library uses TIM14 as Input Capture for zero detection*/

#include "data_types.h"





bool_t getZeroInterruptFlag(void);
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Zero_Interrupt(TIM_HandleTypeDef *htim);
uint16_t getACFrecuency(void);
uint16_t getClockFrecuency(TIM_HandleTypeDef *htim);
void clrZeroFlag(void);
tick_t getDifference(void);

