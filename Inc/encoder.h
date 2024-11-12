#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "tim.h"


//电机1的编码器输入引脚
#define MOTO1_ENCODER1_PORT GPIOA
#define MOTO1_ENCODER1_PIN  GPIO_PIN_0
#define MOTO1_ENCODER2_PORT GPIOA
#define MOTO1_ENCODER2_PIN  GPIO_PIN_1

//定时器号
#define PWM_TIM     htim1
// #define ENCODER_TIM_1 htim2
// #define ENCODER_TIM_2 htim3
// #define ENCODER_TIM_3 htim4
#define GAP_TIM     htim5

#define MOTOR_SPEED_RERATIO 28u    //电机减速比
#define PULSE_PRE_ROUND 500 //一圈多少个脉冲
#define RADIUS_OF_TYRE 34 //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14
#define RELOADVALUE __HAL_TIM_GetAutoreload(htim)    //获取自动装载值,本例中为20000
#define COUNTERNUM __HAL_TIM_GetCounter(htim)        //获取编码器定时器中的计数值

#define SPEED_RECORD_NUM 20 // 经测试，50Hz个采样值进行滤波的效果比较好

typedef struct _Motor
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
    float speed_Record[SPEED_RECORD_NUM]; //速度记录数组
}Motor;

extern Motor motor[3];

void Encoder_Init(Motor *motor,TIM_HandleTypeDef *htim);
float Speed_Low_Filter(float new_Spe,float *speed_Record);
float Motor_getspeed(Motor *motor,TIM_HandleTypeDef *htim);

#endif
