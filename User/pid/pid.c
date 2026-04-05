/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵֺʼPID㺯
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1.
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "math.h"


uint8_t  start_falg;
fp32 fast=1.4;
uint16_t  max_cnt =400;
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PIDṹָ
  * @param[in]      mode: PID_POSITION:ͨPID
  *                 PID_DELTA: PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid
  * @param[in]      max_iout: pid
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

		pid->FAST = fast;
		pid->reference[0] = pid->reference[1] = pid->reference[2] = 0.0f;

}

/**
  * @brief          pid calculate
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid
  * @param[out]     pid: PIDṹָ
  * @param[in]      ref:
  * @param[in]      set: 趨ֵ
  * @retval         pid
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
	if(pid->last_set != set)
	{
		start_falg =1;
		pid->last_set = set;
	}
	if(start_falg)
	{
		if(pid->cnt > max_cnt){
			pid->cnt = 0;
			start_falg = 0;
		}
		else
			pid->cnt++;
	}

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;

//    pid->fdb = ref;
//    pid->error[0] = set - ref;

		pid->reference[2] = pid->reference[1];
    pid->reference[1] = pid->reference[0];
		pid->reference[0] = ref;
		pid->fdb = (pid->reference[0] + pid->reference[1] + pid->reference[2]) / 3.0f;
    pid->error[0] = set - pid->fdb;
		if(!start_falg){
		if(fabs(pid->error[0]) > 100 && pid->cnt == 0&& pid->last_set!=0){
			pid->k= pid->FAST;
			pid->cnt ++;
		}
		else if(pid->k != 1 && pid->cnt < max_cnt/20)
			pid->cnt ++;
		else
			{
			pid->k= 1;
			pid->cnt = 0;
		}
		}
    if (pid->mode == PID_POSITION)
    {
				pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//				if(fabs(pid->Dbuf[0]) >= 800)
//					pid->error[0] = pid->error[1];

        pid->Pout = pid->Kp *pid->k* pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];

        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid
  * @param[out]     pid: PIDṹ֌
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}