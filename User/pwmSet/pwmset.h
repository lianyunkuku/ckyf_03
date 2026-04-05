#ifndef __PWMSET_H__
#define __PWMSET_H__

#include "main.h"
#include "pid.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"


void PWMSet(GPIO_TypeDef *port1,GPIO_TypeDef *port2,uint16_t pin1,uint16_t pin2,uint16_t pwm,pid_type_def *pidType,uint32_t channel);

#endif
