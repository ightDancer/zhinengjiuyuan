#ifndef _PID_H_
#define _PID_H_

#include "encoder.h"
#include <stdio.h>
//#include "control.h"

//PID三个参数的值
#define KP_speed 2
#define KI_speed 0
#define KD_speed 0

#define KP_position 2
#define KI_position 0
#define KD_position 0

#define KP_angel 2;
#define KI_angel 0;
#define KD_angel 0;

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值
    float output,maxOutput;
}PID;

extern PID pid_speed,pid_position,pid_angel;

void PID_Init(void);
float Speed_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
float Location_PID_Realize(PID* pid,float err);
float Angel_PID_Realize(PID* pid,float err);

#endif