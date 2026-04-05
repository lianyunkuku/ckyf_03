#include "ps2.h"


uint16_t Handkey;
uint8_t Comd[2]={0x01,0x42};	//开始命令。请求数据（主机发送到手柄）
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组

uint16_t MASK[]={
    PSB_SELECT, 
    PSB_L3,      
    PSB_R3 ,     
    PSB_START,   
    PSB_PAD_UP,  
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,           
    PSB_R2,                   
    PSB_L1,            
    PSB_R1 ,           
    PSB_GREEN, //三角   
    PSB_RED,   //圆   
    PSB_BLUE,  //叉   
    PSB_PINK   //粉  
	};

	
static inline void delay_us_nos(uint32_t us)
  {
      uint32_t start = DWT->CYCCNT;
      uint32_t cycles = us * (SystemCoreClock / 1000000);
      while((DWT->CYCCNT - start) < cycles);
}	
	
void PS2_Cmd(uint8_t CMD)		  
{
	volatile uint16_t ref=0x01;  //ref=00000001
	Data[1] = 0;   
	for(ref=0x01;ref<0x0100;ref<<=1) // 00000001——>10000000
	{
		if(ref&CMD)    
		{              
			DO_H;             
		}             
		else DO_L;    

		CLK_H;                 
		delay_us_nos(10);          
		CLK_L;                
		delay_us_nos(10);           
		CLK_H;         
		if(DI)
		{Data[1] = ref|Data[1];} 
	}
}
	




	//判断是否为红灯模式
//返回值；0，红灯模式
uint8_t PS2_RedLight(void)
{
	CS_L;   
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;   
	if( Data[1] == 0X73)   return 0 ;//返回0位红灯模式
	else return 1;

}





//读取手柄数据
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据

	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			CLK_L;
			delay_us_nos(10);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us_nos(10);
	}
	
	CS_H;	
}



uint8_t PS2_DataKey()
{
	uint8_t index;  //按键记录值（最终返回值）

	PS2_ClearData();//清除数组数据
	PS2_ReadData(); //将数据读取到

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}



//得到一个摇杆的模拟量	 范围0~256
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];                 
}




//清除数据缓冲区
void PS2_ClearData()
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}

void PS2_ShortPoll(void) //手柄配置初始化
{
	CS_L; delay_us_nos(16); 
  PS2_Cmd(0x01);
	PS2_Cmd(0x42);
  PS2_Cmd(0X00);
	PS2_Cmd(0x00);   //小电机不震动
  PS2_Cmd(0x00);  //大电机不震动
	CS_H;
  delay_us_nos(16); 
}

void PS2_EnterConfing(void) //进入配置
	{ 
		CS_L;
		delay_us_nos(16);
		PS2_Cmd(0x01);
		PS2_Cmd(0x43);
		PS2_Cmd(0X00);
		PS2_Cmd(0x01); 
		PS2_Cmd(0x00); 
		PS2_Cmd(0X00);
		PS2_Cmd(0X00); 
		PS2_Cmd(0X00);
		PS2_Cmd(0X00);
		CS_H;
		delay_us_nos(16);
} 


// 发送模式设置
void PS2_TurnOnAnalogMode(void)
	{ 
		CS_L; 
		PS2_Cmd(0x01);
		PS2_Cmd(0x44);
		PS2_Cmd(0X00); 
		PS2_Cmd(0x01);//0x01红灯  0x00绿灯模式
		PS2_Cmd(0xEE);//Ox03 锁存设置，即不可通过按键“MODE ”设置模式。 
		              //0xEE 不锁存软件设置，可通过按键“MODE ”设置模式。
		PS2_Cmd(0X00); 
	  PS2_Cmd(0X00);
  	PS2_Cmd(0X00); 
	  PS2_Cmd(0X00); 
	  CS_H;
	  delay_us_nos(16); 
	} 

	// 振动设置
	void PS2_VibrationMode(void)
 { 
  CS_L; 
  delay_us_nos(16); 
  PS2_Cmd(0x01);
  PS2_Cmd(0x4D);
  PS2_Cmd(0X00);
  PS2_Cmd(0x00);
  PS2_Cmd(0X01);
  CS_H;
  delay_us_nos(16);
 } 
	
	void PS2_ExitConfing(void)// 完成并保存配置
{
  CS_L; 
	delay_us_nos(16);
	PS2_Cmd(0x01); 
	PS2_Cmd(0x43); 
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A); 
	PS2_Cmd(0x5A);
	CS_H; 
	delay_us_nos(16);
}
	
void PS2_SetInit(void)
	{ 
		PS2_ShortPoll(); 
		PS2_ShortPoll(); 
		PS2_ShortPoll(); 
		PS2_EnterConfing(); // 进入配置模式
		PS2_TurnOnAnalogMode(); // “红绿灯”配置模式，并选择是否保存 
		PS2_VibrationMode(); // 开启震动模式 
		PS2_ExitConfing(); // 完成并保存配置
	} 
	
	
void PS2_Vibration(uint8_t motor1,uint8_t motor2)
	{
    CS_L; 
		delay_us_nos(16); 
		PS2_Cmd(0x01); // 开始命令 
		PS2_Cmd(0x42); // 请求数据 
		PS2_Cmd(0X00); 
		PS2_Cmd(motor1); 
		PS2_Cmd(motor2); 
		PS2_Cmd(0X00); 
		PS2_Cmd(0X00); 
		PS2_Cmd(0X00); 
		PS2_Cmd(0X00); 
		CS_H; delay_us_nos(16);
} 
