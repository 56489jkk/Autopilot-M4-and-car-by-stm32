/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-16     Administrator       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_GCS_GCS_COMMUNICATION_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_GCS_GCS_COMMUNICATION_H_

#include "mavlink.h"

#define AP_MAX_NAME_SIZE 16

enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_ATTITUDE_QUATERNION,
    MSG_LOCATION,
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MEMINFO,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW,
    MSG_RAW_IMU,
    MSG_SCALED_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_SYSTEM_TIME,
    MSG_SERVO_OUT,
    MSG_NEXT_MISSION_REQUEST_WAYPOINTS,
    MSG_NEXT_MISSION_REQUEST_RALLY,
    MSG_NEXT_MISSION_REQUEST_FENCE,
    MSG_NEXT_PARAM,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_SIM_STATE,
    MSG_AHRS2,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
    MSG_TERRAIN,
    MSG_BATTERY2,
    MSG_CAMERA_FEEDBACK,
    MSG_GIMBAL_DEVICE_ATTITUDE_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_MAG_CAL_PROGRESS,
    MSG_MAG_CAL_REPORT,
    MSG_EKF_STATUS_REPORT,
    MSG_LOCAL_POSITION,
    MSG_PID_TUNING,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_WHEEL_DISTANCE,
    MSG_MISSION_ITEM_REACHED,
    MSG_POSITION_TARGET_GLOBAL_INT,
    MSG_POSITION_TARGET_LOCAL_NED,
    MSG_ADSB_VEHICLE,
    MSG_BATTERY_STATUS,
    MSG_AOA_SSA,
    MSG_LANDING,
    MSG_ESC_TELEMETRY,
    MSG_EXTENDED_SYS_STATE, //作为四旋翼的飞行状态指示
    MSG_ORIGIN,
    MSG_HOME,
    MSG_NAMED_FLOAT,
    MSG_AUTOPILOT_VERSION,
    MSG_EFI_STATUS,
    MSG_GENERATOR_STATUS,
    MSG_WINCH_STATUS,
    MSG_WATER_DEPTH,
    MSG_HIGH_LATENCY2,
    MSG_AIS_VESSEL,
    MSG_MCU_STATUS,
    MSG_UAVIONIX_ADSB_OUT_STATUS,
    MSG_ATTITUDE_TARGET,
    MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE,
    MSG_LAST // MSG_LAST must be the last entry in this enum
};


typedef struct
{
    mavlink_statustext_t    msg;
    uint16_t                entry_created_ms;
    uint8_t                 bitmask;
}statustext_t;


enum streams {
    STREAM_RAW_SENSORS,
    STREAM_EXTENDED_STATUS,
    STREAM_RC_CHANNELS,
    STREAM_RAW_CONTROLLER,
    STREAM_POSITION,
    STREAM_EXTRA1,
    STREAM_EXTRA2,
    STREAM_EXTRA3,
    STREAM_PARAMS,
    STREAM_ADSB,
    NUM_STREAMS
};

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_GROUP
};

struct pending_param_request {
    mavlink_channel_t chan;
    int16_t param_index;
    char param_name[16+1];
};

struct pending_param_request req;
uint16_t                    _queued_parameter_index;
uint16_t                    _queued_parameter_count;
uint32_t                    _queued_parameter_send_time_ms;

mavlink_status_t *cstatus[4];
// saveable rate of each stream
extern uint16_t        streamRates[11];

// time we last saw traffic from our GCS
uint32_t _sysid_mygcs_last_seen_time_ms;
mavlink_vision_speed_estimate_t _vision_speed_estimate;
mavlink_vision_position_estimate_t _vision_position_estimate;
mavlink_channel_t           chan;
mavlink_extended_sys_state_t copter_sys;

mavlink_vision_position_estimate_t get_vision_position_estimate(void); //返回当前位置
bool GCS_MAVLINK_try_send_message(const enum ap_message id);
void GCS_MAVLINK_handle_common_message(mavlink_message_t msg);//handle messages which don't require vehicle specific data
// called when valid traffic has been seen from our GCS
void sysid_myggcs_seen(uint32_t ffseen_time_ms);
uint32_t sysid_myggcs_last_seen_time_ms(void);
void GCS_MAVLINK_queued_param_send();
uint16_t get_set_servo_pwm();





//#define MAV_SYS_STATUS_SENSOR_3D_GYRO            0x01
//#define MAV_SYS_STATUS_SENSOR_3D_ACCEL           0x02
//#define MAV_SYS_STATUS_SENSOR_3D_MAG             0x04
//#define MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE  0x08
//#define MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW       0x40
//#define MAV_SYS_STATUS_SENSOR_RC_RECEIVER        0x10000     //RC receiver
//#define MAV_SYS_STATUS_AHRS                      0x200000    //AHRS subsystem health
//#define MAV_SYS_STATUS_SENSOR_BATTERY            0x2000000   //Battery


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_GCS_GCS_COMMUNICATION_H_ */
















