#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

typedef struct{
	volatile int32_t cnt;
	int32_t last_cnt;
	uint8_t last_state;
}Encoder;

void EncodeProcess(GPIO_TypeDef *port,uint16_t pin1,uint16_t pin2,Encoder *encoder);


#endif
