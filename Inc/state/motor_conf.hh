#pragma once

#include "stm32f407xx.h"
#include "tim.h"

namespace RescueRobot
{
    typedef struct _MotorConf {
        _MotorConf(GPIO_TypeDef *port, uint16_t pin_1, uint16_t pin_2, uint16_t channel, TIM_HandleTypeDef *tim_handle)
            : port(port), pin_1(pin_1), pin_2(pin_2), channel(channel), tim_handle(tim_handle)
        {
        }

        GPIO_TypeDef *port;
        uint16_t pin_1;
        uint16_t pin_2;
        uint16_t channel;
        TIM_HandleTypeDef *tim_handle;
    } MotorConf;
}