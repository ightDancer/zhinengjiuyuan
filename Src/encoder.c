#include "encoder.h"

Motor motor[3];
void Encoder_Init(Motor *motor,TIM_HandleTypeDef *htim)
{
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);      //开启编码器定时器
    __HAL_TIM_ENABLE_IT(htim,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
    HAL_TIM_Base_Start_IT(&GAP_TIM);                       //开启100ms定时器中断
    __HAL_TIM_SET_COUNTER(htim, 10000);                //编码器定时器初始值设定为10000
    motor->lastCount = 0;                                   //结构体内容初始化
    motor->totalCount = 0;
    motor->overflowNum = 0;                                  
    motor->speed = 0;
    motor->direct = 0;
    // for (int i = 0; i < SPEED_RECORD_NUM; i++)
    // {
    //     motor->speed_Record[i] = 0;
    // }
}

float Speed_Low_Filter(float new_Spe,float *speed_Record)
{
    float sum = 0.0f;
    //test_Speed = new_Spe;
    for(uint8_t i=SPEED_RECORD_NUM-1;i>0;i--)//将现有数据后移一位
    {
        speed_Record[i] = speed_Record[i-1];
        sum += speed_Record[i-1];
    }
    speed_Record[0] = new_Spe;//第一位是新的数据
    sum += new_Spe;
    //test_Speed = sum/SPEED_RECORD_NUM;
    return sum/SPEED_RECORD_NUM;//返回均值
}

float Motor_getspeed(Motor *motor,TIM_HandleTypeDef *htim)
{
    /**********************************电机测速************************************/
        motor->direct = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);//如果向上计数（正转），返回值为0，否则返回值为1
        motor->totalCount = COUNTERNUM + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        
        if(motor->lastCount - motor->totalCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum++;
            motor->totalCount = COUNTERNUM + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        else if(motor->totalCount - motor->lastCount > 19000) // 在计数值溢出时进行防溢出处理
        {
            motor->overflowNum--;
            motor->totalCount = COUNTERNUM + motor->overflowNum * RELOADVALUE;//一个周期内的总计数值等于目前计数值加上溢出的计数值
        }
        
        motor->speed = (float)(motor->totalCount - motor->lastCount) / (4 * MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND) * 3000;//算得每秒多少转,除以4是因为4倍频
        motor->speed = Speed_Low_Filter(motor->speed,motor->speed_Record);
        motor->lastCount = motor->totalCount; //记录这一次的计数值
        return motor->speed;
}



// /*
//  * 进行速度的平均滤波
//  * 输入新采样到的速度，存放速度的数组，
//  * 返回滤波后的速度
//  */
