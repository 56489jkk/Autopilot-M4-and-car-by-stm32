/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#include "loco_config.h"
#include "Arming.h"
#include "mode_all.h"

/***********************************************************
  *@brief  遥控器控制解锁上锁。
*************************************************************/
void rc_unlock_arming(void)  //摇杆控制解锁上锁
{
    static uint16_t lock_cnt = 0;
    if(channelPos[THR]==LO&&channelPos[YAW]==LO&&channelPos[ROL]==CE)
    {
        if(lock_cnt++ > 500&&(acceIsCalibrationComplete(get_gyro_instance())&&gyroIsCalibrationComplete(get_gyro_instance()))&&(!(is_use_external_sensor(OpticalFlow_id)&&!get_opt_health())))
        {
           motor.state = motor_idle;  //电机怠速
        }
    }

    if(channelPos[THR]==LO&&channelPos[YAW]==HI&&channelPos[ROL]==CE)
    {
        lock_cnt = 0;
        motor.state = motor_off;  //关闭所有电机
    }

    static uint32_t unrun_count = 0;
    float thr_in = get_channel_thr_control_in();
    /* 降落上锁 ，仅支持遥控组件，任务组件靠高度控制组件返回的flag判断执行上锁 */
    if( motor.state != motor_off&&((thr_in <-0.3&&absolute(get_velocity_z_up_cms())< 10&&absolute(get_position_z_up_cm())<15)
            || (get_new_mode()== LAND_Mode&&absolute(get_velocity_z_up_cms())<10&&absolute(get_position_z_up_cm())<15)))  //落地上锁,特殊上锁操作,兼容任务和遥控器
    {
        unrun_count++;
    }
    else
    {
        unrun_count = 0;
    }

    if(unrun_count >= 3000||(get_new_mode()==LAND_Mode&&unrun_count >= 100))                            //油门拉到最低,持续2秒,上锁
    {
        motor.state = motor_off;  //关闭所有电机
        unrun_count = 0;
        lock_cnt = 0;
    }

    attitude_t attitude = get_ahrs_eulerAngles();   //为四元数姿态解算得到的欧拉角  单位:度

    if (absolute(attitude.pitch)>35 || absolute(attitude.roll)>35 ) {
        motor.state = motor_off; //关闭所有电机
    }
    one_button_takeoff_check();


//    if(ros_command.command == 400&&ros_command.param1 == 1)
//    {
//        if(lock_cnt++ > 200&&(acceIsCalibrationComplete(get_gyro_instance())&&gyroIsCalibrationComplete(get_gyro_instance()))&&(!(is_use_external_sensor(Tof_id)&&!get_tof_health())))
//        {
//           motor.state = motor_idle;  //电机怠速
//           set_flight_mode(ROS_Nav_Mode,UNKNOWN);
//        }
//    }
//    if(ros_command.command == 22 && motor.state == motor_idle)
//    {
//           set_flight_mode(ROS_Nav_Mode,UNKNOWN);
//    }
//
//    if(ros_command.command == 400&&ros_command.param1 ==0)
//    {
//        motor.state = motor_off;  //关闭所有电机
//    }
}

uint8_t cmd_unlock_arming(void)  //摇杆控制解锁上锁
{
    static uint16_t arming = 0;
    if((acceIsCalibrationComplete(get_gyro_instance())&&gyroIsCalibrationComplete(get_gyro_instance()))&&(!(is_use_external_sensor(OpticalFlow_id)&&!get_opt_health())))
    {
       //motor.state = motor_idle;  //电机怠速
        arming = 1;
    }
    else {
        arming = 0;
    }

    return  arming;
}

uint8_t one_button_flag=0;
void one_button_takeoff_check(void)
{
    static int  cnt_down=0;

//    if (ros_nav_target.z == 120) {
//        rc_ppm_in[6] = 2000;
//    }
//    else if(ros_nav_target.z == 110) {
//
//    }
//    else {
//        //rc_ppm_in[6] = 1000;
//    }

    if(rc_ppm_in[6]<1200&&rc_ppm_in[4]<1200&&rc_ppm_in[6]>850&&rc_ppm_in[4]>850)
    {
        cnt_down++;
        one_button_flag = 0;
    }

/*    if (rc_ppm_in[6]>1800&&cnt_down>500&&get_new_mode()==FLOWHOLD_Mode
            &&(acceIsCalibrationComplete(get_gyro_instance())&&gyroIsCalibrationComplete(get_gyro_instance()))&&(!(is_use_external_sensor(Tof_id)&&!get_tof_health())))
*/
        if (rc_ppm_in[6]>1800&&cnt_down>500&&get_new_mode()==FLOWHOLD_Mode
                &&(acceIsCalibrationComplete(get_gyro_instance())&&gyroIsCalibrationComplete(get_gyro_instance())))
    {
        one_button_flag = 1;
        cnt_down=0;
    }

}

uint8_t is_one_button_takeoff(void)//
{
    return one_button_flag;
}






