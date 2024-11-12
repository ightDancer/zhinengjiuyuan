#include "forward.h"

void Forward(float M1_speed,float M2_speed,float M3_speed,CAR* car_now)
{
    car_now->vx = M1_speed*2/3 - M2_speed/2 - M3_speed/2;
    car_now->vy = -M2_speed/cbrt(3) + M3_speed/cbrt(3);
    car_now->omega = M1_speed/3 - M2_speed/3 + M3_speed/3;
    car_now->x += car_now->vx;
    car_now->y += car_now->vy;
    car_now->alpha += car_now->omega;
}

void Turn(float vx,float vy,float omega)
{
    M1_SetPWM(Speed_PID_Realize(&pid_speed,vy-omega*L,Motor_getspeed(&motor[0],&htim2)));
    M2_SetPWM(Speed_PID_Realize(&pid_speed,-vx*cbrt(3)/2-vy/2-omega*L,Motor_getspeed(&motor[1],&htim3)));
    M3_SetPWM(Speed_PID_Realize(&pid_speed,vx*cbrt(3)/2-vy/2-omega*L,Motor_getspeed(&motor[2],&htim4)));
}