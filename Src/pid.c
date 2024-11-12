#include "pid.h"
PID pid_speed,pid_position,pid_angel;
/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
void PID_Init(void)//PID参数初始化
{
    pid_speed.err = 0;
    pid_speed.integral = 0;
    pid_speed.maxIntegral = 1000;
    pid_speed.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_speed.lastErr = 0;
    pid_speed.output = 0;
    pid_speed.kp = KP_speed;
    pid_speed.ki = KI_speed;
    pid_speed.kd = KD_speed;

    pid_position.err = 0;
    pid_position.integral = 0;
    pid_position.maxIntegral = 80;
    pid_position.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_position.lastErr = 0;
    pid_position.output = 0;
    pid_position.kp = KP_position;
    pid_position.ki = KI_position;
    pid_position.kd = KD_position;

    pid_angel.err = 0;
    pid_angel.integral = 0;
    pid_angel.maxIntegral = 80;
    pid_angel.maxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM);
    pid_angel.lastErr = 0;
    pid_angel.output = 0;
    pid_angel.kp = KP_angel;
    pid_angel.ki = KI_angel;
    pid_angel.kd = KD_angel;
}

float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
    pid->integral += pid->err;
    
    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i


    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
    return pid->output;
}

float Location_PID_Realize(PID* pid,float err)//一次PID计算
{
     if(pid->err < 0.5 && pid->err > -0.5) pid->err = 0;//pid死区
    pid->err = err;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}

float Angel_PID_Realize(PID* pid,float err)//一次PID计算
{
     if(pid->err < 0.5 && pid->err > -0.5) pid->err = 0;//pid死区
    pid->err = err;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}