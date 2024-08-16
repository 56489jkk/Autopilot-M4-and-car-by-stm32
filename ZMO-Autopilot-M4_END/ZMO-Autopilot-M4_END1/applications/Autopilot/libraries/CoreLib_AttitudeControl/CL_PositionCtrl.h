/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_POSITIONCTRL_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_POSITIONCTRL_H_
//input from pliot
Vector3f _target_velocity;
Vector3f _target_pos;
Vector3f _target_vel;            // velocity target in NEU cm/s calculated by pos_to_rate step
Vector3f _target_accel;          // acceleration target in NEU cm/s/s


Vector3f _desired_vel;           // desired velocity in NEU cm/s
Vector3f _desired_accel;         // desired acceleration in NEU cm/s/s (feed forward)




//output from controller
float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller
float       _yaw_target;            // desired yaw in centi-degrees calculated by position controller
float       _yaw_rate_target;       // desired yaw rate in centi-degrees per second calculated by position controller

// internal variables
float       _dt;                    // time difference (in seconds) between calls from the main program
uint64_t    _last_update_xy_ms;     // system time (in microseconds) since last update_xy_controller call
uint64_t    _last_update_z_ms;      // system time (in microseconds) since last update_z_controller call
float       _vel_max_xy_cms;        // max horizontal speed in cm/s used for kinematic shaping
float       _vel_max_up_cms;        // max climb rate in cm/s used for kinematic shaping

float get_wp_nav_pitch(void);
float get_wp_nav_roll(void);
void set_pos_target_z_from_climb_rate_cm(float vel);
//set position, velocity and acceleration targets
void set_pos_vel_accel_for_pos_ctrl(Vector3f pos,Vector3f vel,Vector3f accel);
void posittion_update_z_controller(void);   //高度控制器
void posittion_update_xy_controller(void);  //水平位置控制器

bool is_active_z_ctrl(void);
bool is_active_xy_ctrl(void);
void init_posittion_z_controller(void);
void init_posittion_xy_controller(void);

// get_lean_angles_to_accel - convert roll, pitch lean angles to NE frame accelerations in cm/s/s
void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float* roll_target, float* pitch_target);
void input_vel_accel_z(float *vel, float accel, bool ignore_descent_limit, bool limit_output);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_ATTITUDECONTROL_CL_POSITIONCTRL_H_ */
