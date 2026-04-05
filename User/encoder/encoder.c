#include "encoder.h"

void EncodeProcess(GPIO_TypeDef *port,uint16_t pin1,uint16_t pin2,Encoder *encoder){
	uint8_t A=HAL_GPIO_ReadPin(port,pin1);
	uint8_t B=HAL_GPIO_ReadPin(port,pin2);
	uint8_t now_state=(A<<1)|B;
	uint8_t dir=(encoder->last_state<<2)|now_state;
	switch(dir)
	{
		case 1:
		case 7:
		case 14:
		case 8:
				encoder->cnt++;
				break;
		case 2:
		case 11:
		case 13:
		case 4:
				encoder->cnt--;
				break;
		default:
				break;
		
	}
	encoder->last_state=now_state;
}