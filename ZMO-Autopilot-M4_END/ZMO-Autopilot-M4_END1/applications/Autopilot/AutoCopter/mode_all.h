/*；
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_
#define APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_
#include "stdbool.h"
#include "loco_config.h"
#include "CL_Vector.h"
enum FlightMode {
    STABILIZE_Mode =     0,  // manual airframe angle with manual throttle
    ACRO_Mode =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD_Mode =      2,  // manual airframe angle with automatic throttle
    AUTO_Mode =          3,  // fully automatic waypoint control using mission commands
    GUIDED_Mode =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER_Mode =        5,  // automatic horizontal acceleration with automatic throttle
    RTL_Mode =           6,  // automatic return to launching point
    CIRCLE_Mode =        7,  // automatic circular flight with automatic throttle
    LAND_Mode =          9,  // automatic landing with horizontal position control
    DRIFT_Mode =        11,  // semi-autonomous position, yaw and throttle control
    SPORT_Mode =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP_Mode =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE_Mode =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD_Mode =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE_Mode =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW_Mode =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB_Mode =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS_Mode = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL_Mode =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD_Mode  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW_Mode    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG_Mode    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID_Mode  =    25,  // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE_Mode =   26,  // Autonomous autorotation
    AUTO_RTL_Mode =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    TURTLE_Mode =       28,  // Flip over after crash
    };


enum ModeReason {
    UNKNOWN = 0,
    RC_COMMAND = 1,
    GCS_COMMAND = 2,
    RADIO_FAILSAFE = 3,
    BATTERY_FAILSAFE = 4,
    GCS_FAILSAFE = 5,
    EKF_FAILSAFE = 6,
    GPS_GLITCH = 7,
    MISSION_END = 8,
    THROTTLE_LAND_ESCAPE = 9,
    FENCE_BREACHED = 10,
    TERRAIN_FAILSAFE = 11,
    BRAKE_TIMEOUT = 12,
    FLIP_COMPLETE = 13,
    AVOIDANCE = 14,
    AVOIDANCE_RECOVERY = 15,
    THROW_COMPLETE = 16,
    TERMINATE = 17,
    TOY_MODE = 18,
    CRASH_FAILSAFE = 19,
    SOARING_FBW_B_WITH_MOTOR_RUNNING = 20,
    SOARING_THERMAL_DETECTED = 21,
    SOARING_THERMAL_ESTIMATE_DETERIORATED = 22,
    VTOL_FAILED_TRANSITION = 23,
    VTOL_FAILED_TAKEOFF = 24,
    FAILSAFE = 25, // general failsafes, prefer specific failsafes over this as much as possible
    INITIALISED = 26,
    SURFACE_COMPLETE = 27,
    BAD_DEPTH = 28,
    LEAK_FAILSAFE = 29,
    SERVOTEST = 30,
    STARTUP = 31,
    SCRIPTING = 32,
    UNAVAILABLE = 33,
    AUTOROTATION_START = 34,
    AUTOROTATION_BAILOUT = 35,
    SOARING_ALT_TOO_HIGH = 36,
    SOARING_ALT_TOO_LOW = 37,
    SOARING_DRIFT_EXCEEDED = 38,
    RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL = 39,
    RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND = 40,
    MISSION_CMD = 41,
    FRSKY_COMMAND = 42,
    FENCE_RETURN_PREVIOUS_MODE = 43,
    QRTL_INSTEAD_OF_RTL = 44,
    AUTO_RTL_EXIT = 45,
    LOITER_ALT_REACHED_QLAND = 46,
    LOITER_ALT_IN_VTOL = 47,
    RADIO_FAILSAFE_RECOVERY = 48,
    QLAND_INSTEAD_OF_RTL = 49,
    DEADRECKON_FAILSAFE = 50,
    };

uint16_t set_mode_cnt;

uint16_t get_mode_cut(void);
bool set_flight_mode(enum FlightMode mode, enum ModeReason reason);
// called at 100hz or more
void update_flight_mode(int16_t new_mode);
uint16_t get_new_mode(void);

//获取飞行员的摇杆期望,单位为角度,并且做幅值限制
void get_pilot_desired_lean_angles(float *roll_out, float *pitch_out, float angle_max);
float get_pilot_desired_yaw_rate(float yaw_in);

//得到处理后的油门期望,范围0 to 1
float get_pilot_desired_throttle(void);
float get_pilot_desired_climb_rate(float throttle_control);




/**********************************************************************************/
/***定高模式***/
void altitude_hold_mode_pid_par_init(void);
void mode_altitude_hold_initialization(void);
void mode_altitude_hold_run(void);          //定高模式  （飞行员控制飞行器角度、油门中位时高度保持）
/***定点模式***/
void position_hold_mode_pid_par_init(void);
void mode_position_hold_initialization(void);
void mode_position_hold_run(void);           //定点模式  （飞行员控制）
/***姿态模式***/
void stabilize_mode_pid_par_init(void);
void mode_stabilize_initialization(void);
void mode_stabilize_run(void);             //自稳模式  (飞行员控制飞行器角度,手动控制油门)
/***指导模式***/
void mode_guided_initialization(void); //指导飞行模式  (飞行员控制)
void mode_guided_run(void); //指导飞行模式  (飞行员控制)
/***降落模式***/
void mode_land_initialization(void);      //初始化飞行参数
void mode_land_run(void);                 //定高模式  （飞行员控制飞行器角度、油门中位时高度保持）//200hz
/***光流定点模式***/
void flow_hold_mode_pid_par_init(void);
void mode_flow_hold_initialization(void);
void mode_flow_hold_run(void);                                      //定点模式  （飞行员控制）
void flowhold_flow_to_angle(Vector2f *bf_angles, bool stick_input); //光流位置与速度信息用于位置控制
Vector3f get_key_fly_position_cm(void); //返回记录一键起飞的位置
/***环形飞行模式***/
void circle_mode_pid_par_init(void);
void mode_circle_initialization(void);
void mode_circle_run(void);

/***ros导航飞行模式***/
void ros_nav_mode_pid_par_init(void);
void mode_ros_nav_initialization(void);
void mode_ros_nav_run(void);

/***寻线飞行模式***/
void line_find_mode_pid_par_init(void);
void mode_line_find_initialization(void);
void mode_line_find_run(void);



void GO_point();
/***蓝牙发送航点***/
void DOWN(void);
/***舵机投放***/



#endif /* APPLICATIONS_AUTOPILOT_AUTOCOPTER_MODE_ALL_H_ */
