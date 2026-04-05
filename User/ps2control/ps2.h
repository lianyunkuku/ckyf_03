#ifndef  __ps2_H
#define  __ps2_H
#include "stm32f1xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "tim.h"
/*
	PB4-->DI
	PA10-->DO
	PA11-->CS
	PB3-->CLK
*/
#define DI    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4) 
#define DO_H  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)     
#define DO_L  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)     
#define CS_H  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)      
#define CS_L  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)     
#define CLK_H  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET)   
#define CLK_L  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET)     

/*按键*/
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

/*模拟量*/
#define PSS_RX          5       //右摇杆X轴
#define PSS_RY          6       //右摇杆Y轴
#define PSS_LX          7       //左摇杆X轴
#define PSS_LY          8       //左摇杆Y轴
static inline void delay_us_nos(uint32_t us);
void PS2_ReadData(void); //读数据
void PS2_Cmd(uint8_t CMD);//发数据		  
void PS2_ClearData(void);	  //清除

void PS2_ShortPoll(void);//配置初始化
void PS2_EnterConfing(void); //进入配置
void PS2_TurnOnAnalogMode(void);//发送模式设置
void PS2_VibrationMode(void);//振动设置
void PS2_ExitConfing(void);//完成并保存配置
void PS2_SetInit(void);//手柄配置初始化


uint8_t PS2_DataKey(void);//获取数据并处理数据	
uint8_t PS2_AnologData(uint8_t button); //获取模拟量
void PS2_Vibration(uint8_t motor1,uint8_t motor2);//振动函数
uint8_t PS2_RedLight(void);//读ID


#endif


