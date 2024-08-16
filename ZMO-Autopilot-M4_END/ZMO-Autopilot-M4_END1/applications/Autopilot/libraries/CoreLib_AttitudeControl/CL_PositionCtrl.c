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
#include "arming.h"
#include "CL_Vector.h"
#include "GCS_Communication.h"
#include "GCS_Copter.h"
#include "mavlink.h"
/**返回位置控制器俯仰的期望**/
float get_wp_nav_pitch()
{
    return  _pitch_target;
}

/**返回位置控制器横滚的期望**/
float get_wp_nav_roll()
{
    return  _roll_target;
}

/**设置期望的升降速度**/
void set_pos_target_z_from_climb_rate_cm(float vel)
{
    _target_velocity.z = vel;

    debug_fg[1] = vel;
}

/**设置位置\速度\加速度期望**/
void set_pos_vel_accel_for_pos_ctrl(Vector3f pos,Vector3f vel,Vector3f accel)
{
    _target_pos = pos;   _desired_vel = vel;  _desired_accel = accel;
}


void input_vel_accel_z(float *vel, float accel, bool ignore_descent_limit, bool limit_output)
{
    float _dt = 0.005f;
    if (ignore_descent_limit) {
        // turn off limits in the negative z direction
        //_limit_vector.z = MAX(_limit_vector.z, 0.0f);
    }

    // calculated increased maximum acceleration and jerk if over speed
    float accel_max_z_cmss = 250;
    float jerk_max_z_cmsss = 500;

    // adjust desired alt if motors have not hit their limits
    update_pos_vel_accel(&_target_pos.z,&_desired_vel.z,_desired_accel.z, _dt,1.0f,pid[z_velocity].target,pid[z_velocity].error);

    shape_vel_accel(*vel, accel,
                    _desired_vel.z,&_desired_accel.z,
                    -limit(accel_max_z_cmss, 0.0f, 750.0f), accel_max_z_cmss,
                    jerk_max_z_cmsss, _dt, limit_output);

    update_vel_accel(vel,accel,_dt, 0.0, 0.0);

}

/**初始化高度位置控制器为当前位置\速度\加速度**/
void init_posittion_z_controller()
{
    pid_reset(&pid[z_position]);
    pid_reset(&pid[z_velocity]);

    pid[z_position].target = get_position_z_up_cm();

    _last_update_z_ms = rt_tick_get();
}

/**如果高度位置控制器在前4次循环中运行过,则返回true**/
bool is_active_z_ctrl()
{
    return ((rt_tick_get() - _last_update_z_ms) <= 20);
}

/**运行高度位置控制器,纠正位置\速度\加速度误差**/
void posittion_update_z_controller() //高度控制器200hz 5ms
{
    _last_update_z_ms = rt_tick_get();
    static  int lock_height = 0;
    if(_target_velocity.z == 0)       //定点模式,速度期望为零才锁定当前高度,即油门为中位
    {
        if(lock_height == 0)                                      //锁定在定点模式下控制刚回中一刻的高度
        {
            float alt_kp = get_velocity_z_up_cms()*0.3f;          //修正摇杆归中是的惯性作用
            lock_height = 1;                                      //锁定
            pid[z_position].target = get_position_z_up_cm() + alt_kp;
        }
        float pos_z_measurement = get_position_z_up_cm();
        pid[z_velocity].target = 1.0f*limit((pid[z_position].target - pos_z_measurement), -100,100); //更新内环期望
    }
    else  //给定上升/下降速度期望
    {
        lock_height = 0;
        pid[z_position].target = 0;
        if (get_position_z_up_cm()<40) {
            pid[z_velocity].target = limit(_target_velocity.z, -30, 200);
        }else {
            pid[z_velocity].target = limit(_target_velocity.z, -200, 200);
        }
    }
    //Vector3f earth_acce = get_navigation_earth_frame_acce();
    pid_rate_updata(&pid[z_velocity], pid[z_velocity].target,get_velocity_z_up_cms());

    if (motor.state != motor_active) {
        pid_reset(&pid[z_position]);
        pid_reset(&pid[z_velocity]);
    }

    float throttle_in = pid[z_velocity].output + 500;

    attitude_set_throttle_out(throttle_in,0,0);
}

/**初始化水平位置控制器为当前位置\速度\加速度**/
void init_posittion_xy_controller()
{
    pid_reset(&pid[x_position]);
    pid_reset(&pid[y_position]);
    pid_reset(&pid[x_velocity]);
    pid_reset(&pid[y_velocity]);

    _target_pos = get_position_neu_cm();

    _last_update_xy_ms = rt_tick_get();
}

/**如果水平位置控制器在前4次循环中运行过,则返回true**/
bool is_active_xy_ctrl()
{
    return ((rt_tick_get() - _last_update_xy_ms) <= 20);
}

/**运行水平位置控制器,纠正位置\速度\加速度误差**/
void posittion_update_xy_controller() //水平位置控制器
{
    _last_update_xy_ms = rt_tick_get();
    //Position Controller
    const Vector3f current_pos = get_position_neu_cm();

    Vector2f vel_target;
    Vector2f accel_target;

    vel_target.x = pid_pos_updata(&pid[x_position],_target_pos.x,current_pos.x);
    vel_target.y = pid_pos_updata(&pid[y_position],_target_pos.y,current_pos.y);

    //Velocity Controller
    const Vector3f current_vel = get_velocity_neu_cms();

    vel_target.x += _desired_vel.x;
    vel_target.y += _desired_vel.y;

    vel_target.x = limit(vel_target.x, -1.0f*200,200);
    vel_target.y = limit(vel_target.y, -1.0f*200,200);

    if(get_pos_vel_type() == 2)//机体坐标下
    {
        accel_target.x =  pid_rate_updata(&pid[x_velocity],vel_target.x,current_vel.x);//前后
        accel_target.y =  -pid_rate_updata(&pid[y_velocity],vel_target.y,current_vel.y);//左右
    }
    else//世界坐标系下
    {
        mavlink_vision_position_estimate_t vision_estimate = get_vision_position_estimate();

        float radar_x =  pid_rate_updata(&pid[x_velocity],vel_target.x,current_vel.x);
        float radar_y =  pid_rate_updata(&pid[y_velocity],vel_target.y,current_vel.y);

        accel_target.y = -sinf(vision_estimate.yaw)*radar_x + cosf(vision_estimate.yaw)*radar_y;//横滚
        accel_target.x =  cosf(vision_estimate.yaw)*radar_x + sinf(vision_estimate.yaw)*radar_y;//俯仰
    }

    accel_to_lean_angles(-accel_target.x,-accel_target.y,&_roll_target,&_pitch_target);
}



// get_lean_angles_to_accel - convert roll, pitch lean angles to NE frame accelerations in cm/s/s
void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float* roll_target, float* pitch_target)
{
    // rotate accelerations into body forward-right frame
    const float accel_forward = accel_x_cmss;
    const float accel_right = accel_y_cmss;

    // update angle targets that will be passed to stabilize controller
    *pitch_target = atanf(accel_forward / (9.8 * 100.0f)) * (180.0f / PI);
    float cos_pitch_target = cosf(*pitch_target * M_PI / 180.0f);
    *roll_target = atanf(accel_right * cos_pitch_target / (9.8 * 100.0f)) * (180.0f / PI);
}



