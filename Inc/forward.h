#ifndef _FORWARD_H_
#define _FORWARD_H_

#include <stdio.h>
#include <math.h>
#include "motorsetpwm.h"
#include "pid.h"
#include "encoder.h"

#define L 25

typedef struct _CAR
{
    float vx,vy,omega;

    float x,y,alpha;
}CAR;

typedef struct _RECEIVE
{
    float miderr;

    float distance;
}RECEIVE;

void Forward(float M1_speed,float M2_speed,float M3_speed,CAR* carspeed);
void Turn(float vx,float vy,float omega);

#endif