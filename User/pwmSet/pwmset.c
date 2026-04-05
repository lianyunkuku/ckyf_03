#include "pwmset.h"

void PWMSet(GPIO_TypeDef *port1,GPIO_TypeDef *port2,uint16_t pin1,uint16_t pin2,uint16_t pwm,pid_type_def *pidType,uint32_t channel){
	if(pidType->out>2){
		__HAL_TIM_SET_COMPARE(&htim2,channel,pwm);
		HAL_GPIO_WritePin(port1,pin1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port2,pin2,GPIO_PIN_SET);
	}else if(pidType->out<-2){
		__HAL_TIM_SET_COMPARE(&htim2,channel,pwm);
		HAL_GPIO_WritePin(port1,pin1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(port2,pin2,GPIO_PIN_RESET);
	}else{
		__HAL_TIM_SET_COMPARE(&htim2,channel,0);
	}
}