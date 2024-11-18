#pragma once

#include <cstdint>
#include <stdint.h>

#include "state/motor_conf.hh"
#include "state/pid.hh"
#include "stm32f407xx.h"

#include "encoder.hh"

namespace RescueRobot
{
    static TIM_HandleTypeDef PWM_TIM = htim1;

    typedef struct _Motor {
        _Motor() = delete;

        _Motor(GPIO_TypeDef *port, uint16_t pin_1, uint16_t pin_2,
               uint16_t channel, TIM_HandleTypeDef *tim_handle,
               float innerKp, float innerKi, float innerKd,
               float innerMaxIntegral,
               float outerKp, float outerKi, float outerKd,
               float outerMaxIntegral, float innerMaxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM),
               float outerMaxOutput = __HAL_TIM_GetAutoreload(&PWM_TIM))
            : conf_(port, pin_1, pin_2, channel, tim_handle),
              pid_(innerKp, innerKi, innerKd, innerMaxIntegral, innerMaxOutput,
                   outerKp, outerKi, outerKd, outerMaxIntegral, outerMaxOutput)
        {
        }

        void init()
        {

            HAL_TIM_Encoder_Start(conf_.tim_handle, TIM_CHANNEL_ALL);
            __HAL_TIM_ENABLE_IT(conf_.tim_handle, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&GAP_TIM);
            __HAL_TIM_SET_COUNTER(conf_.tim_handle, 10000);
        }

        float getSpeed()
        {
            return encoder_.getSpeed(conf_.tim_handle);
        }

        Encoder encoder_;
        MotorConf conf_;
        CascadePID pid_;

    } Motor;
}