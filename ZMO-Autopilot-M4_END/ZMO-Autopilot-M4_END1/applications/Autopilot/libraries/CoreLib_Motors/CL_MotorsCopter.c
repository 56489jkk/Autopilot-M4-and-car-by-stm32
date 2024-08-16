/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#include "usb_transfer.h"
#include "CL_MotorsCopter.h"
#include "loco_config.h"
Motors_t motor;

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */
int pwm_output_init(void)
{
    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return 0;
    }

    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(pwm_dev, 1, 2500000, 1000000-1);
    rt_pwm_set(pwm_dev, 2, 2500000, 1000000-1);
    rt_pwm_set(pwm_dev, 3, 2500000, 1000000-1);
    rt_pwm_set(pwm_dev, 4, 2500000, 1000000-1);

    /*使能设备*/
    rt_pwm_enable(pwm_dev, 1);
    rt_pwm_enable(pwm_dev, 2);
    rt_pwm_enable(pwm_dev, 3);
    rt_pwm_enable(pwm_dev, 4);


    return 0;
}
/* 导出到自动初始化 */
INIT_APP_EXPORT(pwm_output_init);

int  _throttle_out = 0;
void set_motors_throttle(float throttle_in)
{
    _throttle_out = throttle_in;
}

int  _esc_calibrating = 0;
int  get_esc_calibrating_state()
{
   return  _esc_calibrating;
}

#define FINAL_P             0.35f  //
void ctrl_Attitude_MultiRotor_PWM(float outRoll,float outPitch,float outYaw)
{
    if(pwm_dev == RT_NULL)  return ;       //没有设备时跳出
    float roll_out=0,pitch_out=0,yaw_out=0;
    static int inti = 0;
    roll_out  =  outRoll;
    pitch_out = outPitch;
    yaw_out   = outYaw;

    //_throttle_out = rc_ppm_in[2]-1000;
    motor.pwm[0] = limit(_throttle_out + (roll_out*(-1) + pitch_out*(+1) + yaw_out*(+1)) , 40,1000-50);  //m1
    motor.pwm[1] = limit(_throttle_out + (roll_out*(+1) + pitch_out*(-1) + yaw_out*(+1)) , 40,1000-50);  //m2
    motor.pwm[2] = limit(_throttle_out + (roll_out*(+1) + pitch_out*(+1) + yaw_out*(-1)) , 40,1000-50);  //m3
    motor.pwm[3] = limit(_throttle_out + (roll_out*(-1) + pitch_out*(-1) + yaw_out*(-1)) , 40,1000-50);  //m4

    switch (motor.state) {
    case motor_off:

        if(inti == 0)
        {
            rt_pwm_set(pwm_dev, 1, 2500000, 1000000);
            rt_pwm_set(pwm_dev, 2, 2500000, 1000000);
            rt_pwm_set(pwm_dev, 3, 2500000, 1000000);
            rt_pwm_set(pwm_dev, 4, 2500000, 1000000);
            inti = 1;
        }
        break;
    case motor_idle:
        inti=0;
        rt_pwm_set(pwm_dev, 1, 2500000, 1000000+40*1000);
        rt_pwm_set(pwm_dev, 2, 2500000, 1000000+40*1000);
        rt_pwm_set(pwm_dev, 3, 2500000, 1000000+40*1000);
        rt_pwm_set(pwm_dev, 4, 2500000, 1000000+40*1000);
        break;
    case motor_active:
        /*设置PWM周期和脉冲宽度默认值 */
        rt_pwm_set(pwm_dev, MOTOR1_CH, 2500000, 1000000+motor.pwm[0]*1000);
        rt_pwm_set(pwm_dev, MOTOR2_CH, 2500000, 1000000+motor.pwm[1]*1000);
        rt_pwm_set(pwm_dev, MOTOR3_CH, 2500000, 1000000+motor.pwm[2]*1000);
        rt_pwm_set(pwm_dev, MOTOR4_CH, 2500000, 1000000+motor.pwm[3]*1000);
    default:
        break;
   }





    static int ESC_init = 1,esc_cnt=0;
    if (rc_ppm_in[2]>1800&&rc_ppm_in[5]>1800&&rc_ppm_in[7]>1800&&rc_ppm_in[3]>1800&&ESC_init == 1) {

        esc_cnt++;
    }
    if (esc_cnt>2000) {
      if(rc_ppm_in[2]>1800&&rc_ppm_in[5]>1800&&rc_ppm_in[7]>1800&&ESC_init == 1)
       {
           rt_pwm_set(pwm_dev, 1, 2500000, 1000000+1000*1000);
           rt_pwm_set(pwm_dev, 2, 2500000, 1000000+1000*1000);
           rt_pwm_set(pwm_dev, 3, 2500000, 1000000+1000*1000);
           rt_pwm_set(pwm_dev, 4, 2500000, 1000000+1000*1000);
           ESC_init =  2;
           _esc_calibrating = 1;
       }
       else if(rc_ppm_in[2]<1100 && ESC_init ==  2)
       {
           rt_pwm_set(pwm_dev, 1, 2500000, 1000000);
           rt_pwm_set(pwm_dev, 2, 2500000, 1000000);
           rt_pwm_set(pwm_dev, 3, 2500000, 1000000);
           rt_pwm_set(pwm_dev, 4, 2500000, 1000000);
           ESC_init =  3;
           _esc_calibrating = 0;
       }
    }


}



