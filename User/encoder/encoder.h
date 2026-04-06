#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"


/****编码器结构体****/
/*
*cnt：当前编码器计数
*last——cnt：上一次读取时编码器的计数
*last——state：上一次编码器的状态，用于判断正反转
*/
typedef struct{
	volatile int32_t cnt;
	int32_t last_cnt;
	uint8_t last_state;
}Encoder;

void EncodeProcess(GPIO_TypeDef *port,uint16_t pin1,uint16_t pin2,Encoder *encoder);//编码器处理函数


#endif
