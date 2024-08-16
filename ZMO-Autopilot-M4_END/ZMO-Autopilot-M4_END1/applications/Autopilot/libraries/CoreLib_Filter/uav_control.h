#include "stdint.h"
float Uav_model_pid_control_roll( uint16_t rc_roll,float kp ,float ki ,float t );//roll control
float Uav_model_pid_control_pitch( uint16_t rc_pitch,float kp ,float ki ,float t );//pitch control
float Uav_model_pid_control_thr( uint16_t rc_thr,float kp ,float ki ,float t ); //thr control
float Uav_model_pid_control_yaw( uint16_t rc_yaw,float kp ,float ki ,float t );//yaw control

