#ifndef _MOTORSETPWM_H_
#define _MOTORSETPWM_H_

#include "tim.h"
#include "gpio.h"


// typedef struct _PWM
// {

//     float m1;         //电机转速
//     float m2;
//     float m3;

// }PWM;


void Motor_Init(void);
void M1_SetPWM(float pwm);
void M2_SetPWM(float pwm);
void M3_SetPWM(float pwm);
//void Motor_Control(PWM *pwm);

#endif 