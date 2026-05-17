#include <driver_timer.h>

TIM_Object tim14={
	.ClockTime = 0,
	.Test_Time = 0
};

void TIMx_Init(TIM_HandleTypeDef* handle,TIM_ElapsedCallback callback)
{
    if(handle->Instance==TIM14)
		{
		  tim14.Handle=handle;
			tim14.ElapCallback=callback;
		}
}

void TIM_Open(TIM_Object* tim)
{
	HAL_TIM_Base_Start_IT(tim->Handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	
	if(htim==(&htim14))
	{
		tim14.ElapCallback();
	}
}

