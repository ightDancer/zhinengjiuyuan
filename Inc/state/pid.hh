#pragma once

#include "tim.h"

namespace RescueRobot
{

    typedef struct _PID {

        _PID(float kp, float ki, float kd, float maxIntegral, float maxOutput)
        {
            this->kp          = kp;
            this->ki          = ki;
            this->kd          = kd;
            this->maxIntegral = maxIntegral;
            this->maxOutput   = maxOutput;
        }

        void calc(float reference, float feedback)
        {
            lastErr = err;
            err     = reference - feedback;
            if (err <= 0.5f && err >= -0.5f) {
                err = 0;
            }

            integral += err;
            if (integral > maxIntegral)
                integral = maxIntegral;
            else if (integral < -maxIntegral)
                integral = -maxIntegral;

            output = kp * err + ki * integral + kd * (err - lastErr);
            if (output > maxOutput)
                output = maxOutput;
            else if (output < -maxOutput)
                output = -maxOutput;
        }

        float kp, ki, kd;
        float err, lastErr;
        float integral, maxIntegral;
        float output, maxOutput;
    } PID;

    typedef struct _CascadePID {

        /**
         * @brief Construct a new cascade pid object. inner for velocity, outer for position
         *
         */
        _CascadePID(float innerKp, float innerKi, float innerKd,
                    float innerMaxIntegral, float innerMaxOutput,
                    float outerKp, float outerKi, float outerKd,
                    float outerMaxIntegral, float outerMaxOutput)
            : inner_(innerKp, innerKi, innerKd, innerMaxIntegral, innerMaxOutput),
              outer_(outerKp, outerKi, outerKd, outerMaxIntegral, outerMaxOutput)
        {
        }

        /**
         * @brief Calculate the output of the cascade PID
         *
         * @param setpoint  target position
         * @param innerInput actual velocity
         * @param outerInput actual position
         */
        void calc(float setpoint, float innerInput, float outerInput)
        {
            outer_.calc(setpoint, outerInput);
            inner_.calc(outer_.output, innerInput);
            output = inner_.output;
        }

        PID inner_; // inner PID is used to control the velocity
        PID outer_; // outer PID is used to control the position

        float output;
    } CascadePID;

}