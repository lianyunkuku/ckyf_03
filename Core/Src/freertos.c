/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "pid.h"
#include "math.h"
#include "ps2.h"
#include "pwmset.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
pid_type_def posiX,posiY,posiT,speeA,speeB,speeC,speeD;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LineNum 13.0f
#define ReductRate 34.0f
#define WheelDia 68.0f
#define PulsesPerRoll 1768.0f
#define PulsesPermm 1768.0f/(68.0f*PI)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
fp32 kpidX[3]={0.6f,0.0f,0.0f},kpidY[3]={0.6f,0.0f,0.0f},kpidT[3]={0.9f,0.0f,0.0f};
fp32 kpidA[3]={10.0f,0.0f,0.0f},kpidB[3]={10.0f,0.0f,0.0f},kpidC[3]={10.0f,0.0f,0.0f},kpidD[3]={10.0f,0.0f,0.0f};
  uint8_t flag=1,roll_flag=0;
fp32 first_omega=0,d_omega=0;
fp32 x_set=0,y_set=0,t_set=0;
fp32 t_conSpeed=0;
fp32 now_speeA=0,now_speeB=0,now_speeC=0,now_speeD=0;
uint8_t cnt=0,lx=0,ly=0,rx=0,ry=0;
int initValue=1;
int initValue2=1;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MPU6050Task02 */
osThreadId_t MPU6050Task02Handle;
const osThreadAttr_t MPU6050Task02_attributes = {
  .name = "MPU6050Task02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PIDcalculateTas */
osThreadId_t PIDcalculateTasHandle;
const osThreadAttr_t PIDcalculateTas_attributes = {
  .name = "PIDcalculateTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ps2task */
osThreadId_t ps2taskHandle;
const osThreadAttr_t ps2task_attributes = {
  .name = "ps2task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MPUyawDataQueue01 */
osMessageQueueId_t MPUyawDataQueue01Handle;
const osMessageQueueAttr_t MPUyawDataQueue01_attributes = {
  .name = "MPUyawDataQueue01"
};
/* Definitions for MotorASpeedQueue02 */
osMessageQueueId_t MotorASpeedQueue02Handle;
const osMessageQueueAttr_t MotorASpeedQueue02_attributes = {
  .name = "MotorASpeedQueue02"
};
/* Definitions for MotorBSpeedQueue03 */
osMessageQueueId_t MotorBSpeedQueue03Handle;
const osMessageQueueAttr_t MotorBSpeedQueue03_attributes = {
  .name = "MotorBSpeedQueue03"
};
/* Definitions for MotorCSpeedQueue04 */
osMessageQueueId_t MotorCSpeedQueue04Handle;
const osMessageQueueAttr_t MotorCSpeedQueue04_attributes = {
  .name = "MotorCSpeedQueue04"
};
/* Definitions for MotorDSpeedQueue05 */
osMessageQueueId_t MotorDSpeedQueue05Handle;
const osMessageQueueAttr_t MotorDSpeedQueue05_attributes = {
  .name = "MotorDSpeedQueue05"
};
/* Definitions for MotorAPosiQueue07 */
osMessageQueueId_t MotorAPosiQueue07Handle;
const osMessageQueueAttr_t MotorAPosiQueue07_attributes = {
  .name = "MotorAPosiQueue07"
};
/* Definitions for MotorBPosiQueue07 */
osMessageQueueId_t MotorBPosiQueue07Handle;
const osMessageQueueAttr_t MotorBPosiQueue07_attributes = {
  .name = "MotorBPosiQueue07"
};
/* Definitions for MotorCPosiQueue08 */
osMessageQueueId_t MotorCPosiQueue08Handle;
const osMessageQueueAttr_t MotorCPosiQueue08_attributes = {
  .name = "MotorCPosiQueue08"
};
/* Definitions for MotorDPosiQueue09 */
osMessageQueueId_t MotorDPosiQueue09Handle;
const osMessageQueueAttr_t MotorDPosiQueue09_attributes = {
  .name = "MotorDPosiQueue09"
};
/* Definitions for mpuReadTimer01 */
osTimerId_t mpuReadTimer01Handle;
const osTimerAttr_t mpuReadTimer01_attributes = {
  .name = "mpuReadTimer01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MPU6050task02(void *argument);
void PIDTask03(void *argument);
void ps2Task04(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of mpuReadTimer01 */
  mpuReadTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &mpuReadTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MPUyawDataQueue01 */
  MPUyawDataQueue01Handle = osMessageQueueNew (5, sizeof(float), &MPUyawDataQueue01_attributes);

  /* creation of MotorASpeedQueue02 */
  MotorASpeedQueue02Handle = osMessageQueueNew (4, sizeof(float), &MotorASpeedQueue02_attributes);

  /* creation of MotorBSpeedQueue03 */
  MotorBSpeedQueue03Handle = osMessageQueueNew (4, sizeof(float), &MotorBSpeedQueue03_attributes);

  /* creation of MotorCSpeedQueue04 */
  MotorCSpeedQueue04Handle = osMessageQueueNew (4, sizeof(float), &MotorCSpeedQueue04_attributes);

  /* creation of MotorDSpeedQueue05 */
  MotorDSpeedQueue05Handle = osMessageQueueNew (4, sizeof(float), &MotorDSpeedQueue05_attributes);

  /* creation of MotorAPosiQueue07 */
  MotorAPosiQueue07Handle = osMessageQueueNew (4, sizeof(float), &MotorAPosiQueue07_attributes);

  /* creation of MotorBPosiQueue07 */
  MotorBPosiQueue07Handle = osMessageQueueNew (4, sizeof(float), &MotorBPosiQueue07_attributes);

  /* creation of MotorCPosiQueue08 */
  MotorCPosiQueue08Handle = osMessageQueueNew (4, sizeof(float), &MotorCPosiQueue08_attributes);

  /* creation of MotorDPosiQueue09 */
  MotorDPosiQueue09Handle = osMessageQueueNew (4, sizeof(float), &MotorDPosiQueue09_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MPU6050Task02 */
  MPU6050Task02Handle = osThreadNew(MPU6050task02, NULL, &MPU6050Task02_attributes);

  /* creation of PIDcalculateTas */
  PIDcalculateTasHandle = osThreadNew(PIDTask03, NULL, &PIDcalculateTas_attributes);

  /* creation of ps2task */
  ps2taskHandle = osThreadNew(ps2Task04, NULL, &ps2task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MPU6050task02 */
/**
* @brief Function implementing the MPU6050Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPU6050task02 */
void MPU6050task02(void *argument)
{
  /* USER CODE BEGIN MPU6050task02 */
	fp32 yaw=0;
	fp32 a=0,b=0;
	initValue=MPU6050_DMP_init();
	 while(initValue!=0){
			initValue=MPU6050_DMP_init();
	 }
  /* Infinite loop */
  for(;;)
  {
    if(initValue2=MPU6050_DMP_Get_Date(&a,&b,&yaw),MPU6050_DMP_Get_Date(&a,&b,&yaw)==0){
			
			//if(yaw<0)yaw+=360;
			if(cnt<200){
				cnt++;
			}
			osMessageQueuePut(MPUyawDataQueue01Handle,&yaw,0,0);
			vTaskDelay(10);
		}else{
			vTaskDelay(1);
		}
  }
  /* USER CODE END MPU6050task02 */
}

/* USER CODE BEGIN Header_PIDTask03 */
/**
* @brief Function implementing the PIDcalculateTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PIDTask03 */
void PIDTask03(void *argument)
{
  /* USER CODE BEGIN PIDTask03 */

	uint16_t pwm[4]={0};
	fp32 Omega=0;
	fp32 tar_speed[4]={0};//A1��Ӧ��ǰ��B2��Ӧ���C3��Ӧ�Һ�D4��Ӧ��ǰ
	fp32 Acnt=0,Bcnt=0,Ccnt=0,Dcnt=0;
	fp32 x_now=0,x_tar=0,y_now=0,y_tar=0,t_now=0,t_tar=0;
	fp32 x_SetSpeed=0,y_SetSpeed=0,t_SetSpeed=0;
	PID_init(&posiX,PID_POSITION,&kpidX[0],1000,200);
	PID_init(&posiY,PID_POSITION,&kpidY[0],1000,200);
	PID_init(&posiT,PID_POSITION,&kpidT[0],50,200);
	PID_init(&speeA,PID_POSITION,&kpidA[0],999,50);
  PID_init(&speeB,PID_POSITION,&kpidB[0],999,50);
	PID_init(&speeC,PID_POSITION,&kpidC[0],999,50);
	PID_init(&speeD,PID_POSITION,&kpidD[0],999,50);
  /* Infinite loop */
  for(;;)
  {
		if(osMessageQueueGet(MPUyawDataQueue01Handle,&Omega,NULL,0)!=osOK){
			osDelay(1);
		}
    if(osMessageQueueGet(MotorASpeedQueue02Handle,&now_speeA,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorBSpeedQueue03Handle,&now_speeB,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorCSpeedQueue04Handle,&now_speeC,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorDSpeedQueue05Handle,&now_speeD,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorAPosiQueue07Handle,&Acnt,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorBPosiQueue07Handle,&Bcnt,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorCPosiQueue08Handle,&Ccnt,NULL,0)!=osOK){
			osDelay(1);
		}
		if(osMessageQueueGet(MotorDPosiQueue09Handle,&Dcnt,NULL,0)!=osOK){
			osDelay(1);
		}
		if(flag==1&&cnt==200){
			first_omega=Omega;
			flag=0;
		}
		if(first_omega>=360){
			first_omega-=360;
		}else if(first_omega<0){
			first_omega+=360;
		}
		if(cnt==200){
			d_omega=first_omega-Omega;
		}
		/*******PS2 Remote Control Part*******/
		
		
		
		
		
		/**************************************/
			 if(t_conSpeed != 0){
      // 正在手动旋转，更新目标角度，禁用位置环
      first_omega = Omega;
      d_omega = 0;
      
			}else{
      // 没有手动旋转，计算位置环
      
      while(d_omega<-180){d_omega+=360;}
      while(d_omega>180){d_omega-=360;}
      
			}
			x_SetSpeed=PID_calc(&posiX,0,x_set);//x(-230-230)
			y_SetSpeed=PID_calc(&posiY,0,y_set);//y(-230-230)
			t_SetSpeed=PID_calc(&posiT,0,d_omega);//T(-540-540)
			
			tar_speed[0]=x_SetSpeed+y_SetSpeed+t_SetSpeed+t_conSpeed;
			tar_speed[1]=x_SetSpeed-y_SetSpeed+t_SetSpeed+t_conSpeed;
			tar_speed[2]=x_SetSpeed+y_SetSpeed-t_SetSpeed-t_conSpeed;
			tar_speed[3]=x_SetSpeed-y_SetSpeed-t_SetSpeed-t_conSpeed;
			pwm[0]=(uint16_t)fabs(PID_calc(&speeA,-now_speeA,tar_speed[0]));
			pwm[1]=(uint16_t)fabs(PID_calc(&speeB,-now_speeB,tar_speed[1]));
			pwm[2]=(uint16_t)fabs(PID_calc(&speeC,-now_speeC,tar_speed[2]));
			pwm[3]=(uint16_t)fabs(PID_calc(&speeD,-now_speeD,tar_speed[3]));
			for(int i=0;i<4;i++){
				if(pwm[i]>999){
					pwm[i]=999;
				}
			}
		
			PWMSet(AIN1_GPIO_Port,AIN2_GPIO_Port,AIN2_Pin,AIN1_Pin,pwm[0],&speeA,TIM_CHANNEL_1);
			PWMSet(BIN1_GPIO_Port,BIN2_GPIO_Port,BIN1_Pin,BIN2_Pin,pwm[1],&speeB,TIM_CHANNEL_2);
			PWMSet(CIN1_GPIO_Port,CIN2_GPIO_Port,CIN1_Pin,CIN2_Pin,pwm[2],&speeC,TIM_CHANNEL_3);
			PWMSet(DIN1_GPIO_Port,DIN2_GPIO_Port,DIN1_Pin,DIN2_Pin,pwm[3],&speeD,TIM_CHANNEL_4);
			vTaskDelay(10);
		
	}
  /* USER CODE END PIDTask03 */
}

/* USER CODE BEGIN Header_ps2Task04 */
/**
* @brief Function implementing the ps2task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ps2Task04 */
void ps2Task04(void *argument)
{
  /* USER CODE BEGIN ps2Task04 */
	uint8_t mode=0,key=0;
	fp32 T_add=0;
  /* Infinite loop */
  for(;;)
  {
    key=PS2_DataKey();
		if(key==PSB_SELECT){
			mode=!mode;
		}
		if(mode==0){
			switch(key){
				case PSB_PAD_UP:
					x_set=100;
					break;
				case PSB_PAD_DOWN:
					x_set=-100;
					break;
				case PSB_PAD_LEFT:
					y_set=-100;
					break;
				case PSB_PAD_RIGHT:
					y_set=100;
					break;
				case PSB_RED:
					t_conSpeed=30;
				
					break;
				case PSB_PINK:
					t_conSpeed=-30;
				
					break;
				default:
					x_set=0;
					y_set=0;
					t_conSpeed=0;
				
					break;
			}
		}else{
			lx=PS2_AnologData(PSS_LX);
			ly=PS2_AnologData(PSS_LY);
			rx=PS2_AnologData(PSS_RX);
			x_set=(fp32)lx-128;
			y_set=(fp32)ly-128;
			t_conSpeed=(fp32)rx-128;
			if(fabs(x_set)<10){x_set=0;}
			if(fabs(y_set)<10){y_set=0;}
			if(fabs(t_conSpeed)<10){t_conSpeed=0;
				
			}
			
			x_set=x_set*100/128;
			y_set=y_set*100/128;
			t_conSpeed=t_conSpeed*100/128;
			}
			vTaskDelay(20);
		}
		
 }
  /* USER CODE END ps2Task04 */


/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */
	
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

