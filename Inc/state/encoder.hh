#pragma once

#include "tim.h"

namespace RescueRobot
{

    constexpr uint8_t SPEED_RECORD_NUM         = 20;
    constexpr uint8_t MOTOR_SPEED_REDUCE_RATIO = 28u;
    constexpr uint16_t PULSE_PRE_ROUND         = 500;
    constexpr uint8_t RADIUS                   = 34; // mm
    constexpr float GIRTH                      = 2 * RADIUS * 3.14;

    static TIM_HandleTypeDef GAP_TIM = htim5;

    inline float Speed_Low_Filter(float new_Spe, float *speed_Record)
    {
        float sum = 0.0f;
        for (uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) {
            speed_Record[i] = speed_Record[i - 1];
            sum += speed_Record[i - 1];
        }
        speed_Record[0] = new_Spe;
        sum += new_Spe;
        return sum / SPEED_RECORD_NUM;
    }

    typedef struct _Encoder {
        float getSpeed(TIM_HandleTypeDef *tim_handle)
        {

            direct_     = __HAL_TIM_IS_TIM_COUNTING_DOWN(tim_handle);
            totalCount_ = __HAL_TIM_GetCounter(tim_handle) +
                          overflowNum_ * __HAL_TIM_GetAutoreload(tim_handle);
            if (lastCount_ - totalCount_ > 19000) {
                overflowNum_++;
                totalCount_ = __HAL_TIM_GetCounter(tim_handle) +
                              overflowNum_ * __HAL_TIM_GetAutoreload(tim_handle);
            } else if (totalCount_ - lastCount_ > 19000) {
                overflowNum_--;
                totalCount_ = __HAL_TIM_GetCounter(tim_handle) +
                              overflowNum_ * __HAL_TIM_GetAutoreload(tim_handle);
            }

            speed_     = (float)(totalCount_ - lastCount_) / (4 * GIRTH * PULSE_PRE_ROUND) * 3000;
            speed_     = Speed_Low_Filter(speed_, speed_Record);
            lastCount_ = totalCount_;
            return speed_;
        }

        int32_t lastCount_{0};
        int32_t totalCount_{0};
        int16_t overflowNum_{0};
        float speed_{0};
        uint8_t direct_{0};
        float speed_Record[SPEED_RECORD_NUM] = {0};
    } Encoder;
}