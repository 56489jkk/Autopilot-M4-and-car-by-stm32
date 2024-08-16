/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MOTORS_CL_MOTORSCOPTER_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MOTORS_CL_MOTORSCOPTER_H_

typedef struct
{
    uint16_t pwm[8];
    uint8_t  state;

}Motors_t;

enum Motors_state {
        motor_off = 0,
        motor_idle = 1,
        motor_active = 2,
        motor_NumState
    };

extern Motors_t motor;
extern int  _throttle_out;
void ctrl_Attitude_MultiRotor_PWM(float outRoll,float outPitch,float outYaw);

void set_motors_throttle(float throttle_in);

int  get_esc_calibrating_state(void);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MOTORS_CL_MOTORSCOPTER_H_ */
