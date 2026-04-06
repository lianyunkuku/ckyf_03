#include "encoder.h"


/****编码器处理函数****/
/*
*port：编码器e引脚对应端口
*pin1，pin2：ExA，ExB对应引脚
*encoder：编码器结构体
*/
void EncodeProcess(GPIO_TypeDef *port,uint16_t pin1,uint16_t pin2,Encoder *encoder){
	uint8_t A=HAL_GPIO_ReadPin(port,pin1);
	uint8_t B=HAL_GPIO_ReadPin(port,pin2);//读取ExA，ExB引脚当前电平
	
	uint8_t now_state=(A<<1)|B;//计算当前电机状态：00，01，11，10，00
	
	uint8_t dir=(encoder->last_state<<2)|now_state;//根据这一次和上一次编码器的状态确定正反转
	switch(dir)
	{
		case 1://0001
		case 7://0111
		case 14://1110
		case 8://1000
				encoder->cnt++;
				break;
		case 2://0010
		case 11://1011
		case 13://1101
		case 4://0100
				encoder->cnt--;
				break;
		default:
				break;
		
	}
	encoder->last_state=now_state;//更新上一次编码器状态
}