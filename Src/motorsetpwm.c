#include "motorsetpwm.h"
//PID pid_speed;

void Motor_Init(void)
{
    HAL_GPIO_WritePin(GPIOE, GND_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GND_M2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GND_M3_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

/**********************************
 * 功能：第n个motor速度开环pwm控制
 * 输入：pwm占空比,范围0-1000
 * 返回：无
 * *******************************/

void M1_SetPWM(float pwm)//PID参数初始化
{
    if(pwm>=0)      //pwm>=0 (IN1, IN2)=(0, 1) 正转 顺时针
    {
        HAL_GPIO_WritePin(GPIOE, IN1_1_Pin, GPIO_PIN_RESET);  //IN1=0
        HAL_GPIO_WritePin(GPIOE, IN1_2_Pin, GPIO_PIN_SET);    //IN2=1
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    }
    else if(pwm<0)  //pwm<0 (IN1, IN2)=(1, 0) 反转 逆时针
    {
        HAL_GPIO_WritePin(GPIOE, IN1_1_Pin, GPIO_PIN_SET); //IN1=1
        HAL_GPIO_WritePin(GPIOE, IN1_2_Pin, GPIO_PIN_RESET); //IN2=0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pwm);
    }
}

void M2_SetPWM(float pwm)//PID参数初始化
{
    if(pwm>=0)      //pwm>=0 (IN1, IN2)=(0, 1) 正转 顺时针
    {
        HAL_GPIO_WritePin(GPIOB, IN2_1_Pin, GPIO_PIN_RESET);  //IN1=0
        HAL_GPIO_WritePin(GPIOB, IN2_2_Pin, GPIO_PIN_SET);    //IN2=1
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    }
    else if(pwm<0)  //pwm<0 (IN1, IN2)=(1, 0) 反转 逆时针
    {
        HAL_GPIO_WritePin(GPIOB, IN2_1_Pin, GPIO_PIN_SET); //IN1=1
        HAL_GPIO_WritePin(GPIOB, IN2_2_Pin, GPIO_PIN_RESET); //IN2=0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pwm);
    }
}

void M3_SetPWM(float pwm)//PID参数初始化
{
    if(pwm>=0)      //pwm>=0 (IN1, IN2)=(0, 1) 正转 顺时针
    {
        HAL_GPIO_WritePin(GPIOB, IN3_1_Pin, GPIO_PIN_RESET);  //IN1=0
        HAL_GPIO_WritePin(GPIOB, IN3_2_Pin, GPIO_PIN_SET);    //IN2=1
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
    }
    else if(pwm<0)  //pwm<0 (IN1, IN2)=(1, 0) 反转 逆时针
    {
        HAL_GPIO_WritePin(GPIOB, IN3_1_Pin, GPIO_PIN_SET); //IN1=1
        HAL_GPIO_WritePin(GPIOB, IN3_2_Pin, GPIO_PIN_RESET); //IN2=0
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pwm);
    }
}

// void Motor_Control(PWM *pwm)
// {
//     M1_SetPWM(pwm->m1);
//     M2_SetPWM(pwm->m2);
//     M3_SetPWM(pwm->m3);
// }

/****************************************
 * 作用：速度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
// float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
// {
//     pid->err = target - feedback;
//     if(pid->err < 0.3 && pid->err > -0.3) pid->err = 0;//pid死区
//     pid->integral += pid->err;
    
//     if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
//     else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

//     if(target == 0) pid->integral = 0; // 刹车时清空i


//     pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

//     //输出限幅
//     if(target >= 0)//正转时
//     {
//         if(pid->output < 0) pid->output = 0;
//         else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
//     }
//     else if(target < 0)//反转时
//     {
//         if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
//         else if(pid->output > 0) pid->output = 0;
//     }

//     pid->lastErr = pid->err;
//     if(target == 0) pid->output = 0; // 刹车时直接输出0
//     return pid->output;
// }
