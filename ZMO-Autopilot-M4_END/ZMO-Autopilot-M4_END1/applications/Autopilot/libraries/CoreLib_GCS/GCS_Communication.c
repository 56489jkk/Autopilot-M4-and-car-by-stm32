/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-16     Administrator       the first version
 */
#include "GCS_Communication.h"
#include "DL_InertialSensor.h"
#include "DL_CompassSensor.h"
#include "mavlink.h"
#include "AutoCopter.h"
#include "mode_all.h"
#include "GCS_Copter.h"
uint16_t streamRates[11]={0};

/****************************************Send MAVLink message***************************************************/
/*发送飞控的心跳包消息*/
void GCS_MAVLINK_send_heartbeat()
{
    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        Copter_base_mode(),
        get_new_mode(),
        3);
}

/*发送飞控核心imu数据*/
void GCS_MAVLINK_send_raw_imu()
{
    Vector3f gyro_deg = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s
    Vector3f acce_g =   get_imu_sensorsAcce();   //为原始数据经第一次低通滤波后的加速度数据 cm/ss
    Vector3f mag = get_imu_sensorsMagn();

    mavlink_msg_raw_imu_send(
        chan,
        0,
        acce_g.x,
        acce_g.y,
        acce_g.z,
        gyro_deg.x,
        gyro_deg.y,
        gyro_deg.z,
        mag.x,
        mag.y,
        mag.z,
        0,  // we use SCALED_IMU and SCALED_IMU2 for other IMUs
        30*100);
}

/*发送飞控第instance个imu模组数据*/
void GCS_MAVLINK_send_scaled_imu(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature))
{
    Vector3f gyro_deg = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s
    Vector3f acce_g =   get_imu_sensorsAcce();   //为原始数据经第一次低通滤波后的加速度数据 cm/ss
    Vector3f mag;

    send_fn(
            chan,
            0,
            acce_g.x ,
            acce_g.y ,
            acce_g.z ,
            gyro_deg.x * 10.0f,
            gyro_deg.y * 10.0f,
            gyro_deg.z * 10.0f,
            mag.x,
            mag.y,
            mag.z,
            45*100);
}

/*发送飞控的姿态数据*/
void GCS_MAVLINK_send_attitude()
{
    //mavlink_vision_position_estimate_t vision_estimate = get_vision_position_estimate();
    const attitude_t attitude = get_ahrs_eulerAngles();//为四元数姿态解算得到的欧拉角  单位：度
    const Vector3f gyro_deg = get_imu_sensorsGyro();   //为原始数据经第一次低通滤波后的角速度数据 deg/s
    mavlink_msg_attitude_send(
        chan,
        0,
        attitude.roll/57.3,
        attitude.pitch/57.3,
        attitude.yaw/57.3,
        gyro_deg.x,
        gyro_deg.y,
        gyro_deg.z);
}

/*发送飞控的气压计数据*/
void GCS_MAVLINK_send_scaled_pressure_instance(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff))
{
    float press_abs = 0.0f;
    float press_diff = 0; // pascal
    int16_t temperature = 0; // Absolute pressure temperature
    int16_t temperature_press_diff = 0; // Differential pressure temperature

    temperature = spl0601_get_temperature();
    press_abs = spl0601_get_pressure()/100;

    send_fn(
        chan,
        0,
        press_abs, // hectopascal
        press_diff, // hectopascal
        temperature, // 0.01 degrees C
        temperature_press_diff); // 0.01 degrees C
}

/*发送飞控的位置信息*/
void GCS_MAVLINK_send_local_position()
{

    Vector3f local_position, velocity;

    local_position = get_position_neu_cm();
    velocity = get_velocity_neu_cms();

    mavlink_msg_local_position_ned_send(
        chan,
        0,
        local_position.x,
        local_position.y,
        local_position.z,
        velocity.x,
        velocity.y,
        velocity.z);
}

/*发送飞控读取的光流传感器数据*/
void GCS_MAVLINK_send_opticalflow()
{
    if (!get_opt_health()) {
        return;
    }
    // get rates from sensor
    Vector2f flowRate ;
    Vector2f bodyRate ;
    flowRate = get_opt_radians();
    bodyRate.x = 0;
    bodyRate.y = 0;

    // populate and send message
    mavlink_msg_optical_flow_send(
        chan,
        0,
        0,                        // sensor id is zero
        flowRate.x,               //dpix
        flowRate.y,               //dpix
        flowRate.x - bodyRate.x,  //m/s
        flowRate.y - bodyRate.y,  //m/s
        0,                        //Optical flow quality / confidence. 0: bad, 255: maximum quality
        0,                        //Ground distance       m
        flowRate.x,               //rad/s
        flowRate.y);              //rad/s

}

void GCS_MAVLINK_send_distance_sensor()
{
    mavlink_msg_distance_sensor_send(
           chan,
           0,                 // time since system boot TODO: take time of measurement
           2,                 // minimum distance the sensor can measure in centimeters
           300,               // maximum distance the sensor can measure in centimeters
           get_opt_distance(),     // current distance reading
           0,                 // type from MAV_DISTANCE_SENSOR enum
           0,                                // onboard ID of the sensor == instance
           0,                   // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
           0,                                       // Measurement covariance in centimeters, 0 for unknown / invalid readings
           0,                                       // horizontal FOV
           0,                                       // vertical FOV
           0,                  // quaternion of sensor orientation for MAV_SENSOR_ROTATION_CUSTOM
           0);                                // Signal quality of the sensor. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.
    mavlink_msg_distance_sensor_send(
           chan,
           0,                        // time since system boot TODO: take time of measurement
           2,               // minimum distance the sensor can measure in centimeters
           300,               // maximum distance the sensor can measure in centimeters
           tfdata.distance,   // current distance reading
           0,                 // type from MAV_DISTANCE_SENSOR enum
           1,                                // onboard ID of the sensor == instance
           0,                   // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
           0,                                       // Measurement covariance in centimeters, 0 for unknown / invalid readings
           0,                                       // horizontal FOV
           0,                                       // vertical FOV
           0,                  // quaternion of sensor orientation for MAV_SENSOR_ROTATION_CUSTOM
           0);
}
/*发送飞控的位置信息*/
void GCS_MAVLINK_send_global_position_int()
{
    Vector3f vel = get_velocity_neu_cms();
    float  alt = get_baro_sensorsAltitude()*10; //mm
    float  relative_alt = get_position_z_up_cm()*10; //mm
    mavlink_msg_global_position_int_send(
        chan,
        0,
        34.4828*1e7,                   // in 1E7 degrees
        113.3023*1e7,                  // in 1E7 degrees
        alt,                           // millimeters above ground/sea level
        relative_alt,                  // millimeters above home
        vel.x * 100,                    // X speed cm/s (+ve North)
        vel.y * 100,                    // Y speed cm/s (+ve East)
        vel.z * 100,                    // Z speed cm/s (+ve Down)
        0);                             // compass heading in 1/100 degree

}

/*发送飞控home点的经纬度及位置信息*/
void GCS_MAVLINK_send_home_position()
{
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mavlink_msg_home_position_send(
        chan,
        0,
        0,
        0* 10,
        0.0f, 0.0f, 0.0f,
        q,
        0.0f, 0.0f, 0.0f,
        0);
}

/*发送飞控读取的GPS信息*/
void GCS_MAVLINK_send_gps_raw(mavlink_channel_t chan)
{
    //const Location &loc = location(0);
    float hacc = 0.0f;
    float vacc = 0.0f;
    float sacc = 0.0f;
    //float undulation = 0.0;
    int32_t height_elipsoid_mm = 0;

    mavlink_msg_gps_raw_int_send(
        chan,
        0,
        0,
        0,        // in 1E7 degrees
        0,        // in 1E7 degrees
        0 * 10UL, // in mm
        0,
        0,
        0*100,  // cm/s
        0*100, // 1/100 degrees,
        0,
        height_elipsoid_mm,   // Elipsoid height in mm
        hacc * 1000,          // one-sigma standard deviation in mm
        vacc * 1000,          // one-sigma standard deviation in mm
        sacc * 1000,          // one-sigma standard deviation in mm/s
        0,                    //
        0);
}

/*发送飞控读取的遥控器原始数据*/
void GCS_MAVLINK_send_rc_channels()
{
    uint16_t values[18] = {};
    //rc().get_radio_in(values, ARRAY_SIZE(values));
    values[0] = rc_ppm_in[0];
    values[1] = rc_ppm_in[1];
    values[2] = rc_ppm_in[2];
    values[3] = rc_ppm_in[3];
    values[4] = rc_ppm_in[4];
    values[5] = rc_ppm_in[5];
    values[6] = rc_ppm_in[6];
    values[7] = rc_ppm_in[7];
    mavlink_msg_rc_channels_send(
        chan,
        0,
        8,
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7],
        values[8],
        values[9],
        values[10],
        values[11],
        values[12],
        values[13],
        values[14],
        values[15],
        values[16],
        values[17],
        0);
}

/*发送飞控各个pwm输出通道的值*/
void GCS_MAVLINK_send_servo_output_raw()
{
    static const uint8_t max_channels = 12;

    uint16_t values[16]={0};
    values[0] = 1500;
    for (uint8_t i=0; i<max_channels; i++) {
        if (values[i] == 65535) {
            values[i] = 0;
        }
    }
    if ((1 & 0xFFFF) != 0) {
        mavlink_msg_servo_output_raw_send(
                chan,
                0,
                0,     // port
                values[0],  values[1],  values[2],  values[3],
                values[4],  values[5],  values[6],  values[7],
                values[8],  values[9],  values[10], values[11],
                values[12], values[13], values[14], values[15]);
    }
}

void GCS_MAVLINK_send_sys_status()
{
    uint32_t control_sensors_present=0;  //存在的
    uint32_t control_sensors_enabled=0;  //使能的
    uint32_t control_sensors_health=0;   //健康的
    control_sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO|MAV_SYS_STATUS_SENSOR_3D_ACCEL|MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE|
                              MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW|MAV_SYS_STATUS_SENSOR_RC_RECEIVER|MAV_SYS_STATUS_PREARM_CHECK;
    control_sensors_enabled = control_sensors_present; //存在的传感器全部使能。

    if(gyroIsCalibrationComplete(get_gyro_instance())){
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if(acceIsCalibrationComplete(get_gyro_instance())){
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }
    if(baroIsCalibrationComplete()){
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if(get_opt_health()){
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
    if(get_rc_health()){
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        0,
        0,
        -1,
        -1,
        0,  // comm drops %,
        0,  // comm drops in pkts,
        0,
        0,
        0,  // errors3
        0); // errors4
}

void GCS_MAVLINK_send_battery_status()
{
    uint16_t cell_mvolts[10]={0};    //mv
    uint16_t cell_current = get_adc_power_current()*100;         //cA
    cell_mvolts[0] = get_adc_power_voltage()*1000; //单位：V
    mavlink_msg_battery_status_send(chan,
                                    0, // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // function
                                    MAV_BATTERY_TYPE_UNKNOWN, // type
                                    25*100,       // temperature. INT16_MAX if unknown
                                    cell_mvolts,  // cell voltages
                                    cell_current,       // current in centiampere
                                    0, // total consumed current in milliampere.hour
                                    0,  // consumed energy in hJ (hecto-Joules)
                                    60,
                                    0, // time remaining, seconds
                                    0, // battery charge state
                                    0, // Cell 11..14 voltages
                                    0, // battery mode
                                    0);   // fault_bitmask
}



/*向地面站发送文本消息*/
char statustext_printf_buffer[256+1];
#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))
#if HAL_MEM_CLASS <= HAL_MEM_CLASS_192 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static const uint8_t _status_capacity = 7;
#else
    static const uint8_t _status_capacity = 30;
#endif
/*
    send a statustext text string to specific MAVLink bitmask
*/
void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t dest_bitmask)
{
    char first_piece_of_text[50+1]={0};
    statustext_t statustext;
    do {
        // send_text can be called from multiple threads; we must
        // protect the "text" member with _statustext_sem
        vsnprintf(statustext_printf_buffer, sizeof(statustext_printf_buffer), fmt, arg_list);
        memcpy(first_piece_of_text, statustext_printf_buffer, ARRAY_SIZE(first_piece_of_text)-1);

        // filter destination ports to only allow active ports.

//        if (update_send_has_been_called) {
//            statustext.bitmask = statustext_send_channel_mask();
//        } else {
//            // we have not yet initialised the streaming-channel-mask,
//            // which is done as part of the update() call.  So just send
//            // it to all channels:
//            statustext.bitmask = (1<<_num_gcs)-1;
//        }
//        statustext.bitmask &= dest_bitmask;
//        if (!statustext.bitmask) {
//            // nowhere to send
//            break;
//        }

        statustext.entry_created_ms = rt_tick_get();
        statustext.msg.severity = severity;

        static uint16_t msgid;
        if (strlen(statustext_printf_buffer) > sizeof(statustext.msg.text)) {
            msgid++;
            if (msgid == 0) {
                msgid = 1;
            }
            statustext.msg.id = msgid;
        }

        const char *remainder = statustext_printf_buffer;
        for (uint8_t i=0; i<_status_capacity; i++) {
            statustext.msg.chunk_seq = i;
            const size_t remainder_len = strlen(remainder);
            // note that remainder_len may be zero here!
            uint16_t n = MIN(sizeof(statustext.msg.text), remainder_len);
            if (i == _status_capacity -1 && n == sizeof(statustext.msg.text)) {
                // fantastic.  This us a very long statustext and
                // we've actually managed to push everything else out
                // of the queue - this is the last chunk, so we MUST
                // null-terminate.
                n -= 1;
            }
            memset(statustext.msg.text, '\0', sizeof(statustext.msg.text));
            memcpy(statustext.msg.text, remainder, n);
            // The force push will ensure comm links do not block other comm links forever if they fail.
            // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
            // block but not until the buffer fills up.
            //_statustext_queue.push_force(statustext);
            remainder = &remainder[n];

            // note that remainder_len here is the remainder length for
            // the *old* remainder!
            if (remainder_len < sizeof(statustext.msg.text) || statustext.msg.id == 0) {
                break;
            }
        }
    } while (false);

    // push the messages out straight away until the vehicle states
    // that it is initialised.  At that point we can assume
    // update_send is being called

    mavlink_msg_statustext_send(chan,
                                statustext.msg.severity,
                                statustext.msg.text,
                                statustext.msg.id,
                                statustext.msg.chunk_seq);
}
void send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list, (1<<chan));
    va_end(arg_list);
}
//send_text(MAV_SEVERITY_INFO,"sysid=%u",1);//例子

/*发送飞控的版本信息*/
void GCS_MAVLINK_send_autopilot_version()
{
    uint8_t  uid2[18] = {0};

    mavlink_msg_autopilot_version_send(
        chan,
        1,
        1111,
        1112,
        1113,
        1114,
        0,
        0,
        0,
        111,
        112,
        1115,
        uid2
    );
}

//Send the main MAVLINK message
bool GCS_MAVLINK_try_send_message(const enum ap_message id)
{
    bool ret = true;
    switch(id) {

    case MSG_SYS_STATUS:
        GCS_MAVLINK_send_sys_status();
        break;

    case MSG_ATTITUDE:
        GCS_MAVLINK_send_attitude();
        break;

    case MSG_ATTITUDE_QUATERNION:

        break;

    case MSG_NEXT_PARAM:

        //queued_param_send();
        break;

    case MSG_HEARTBEAT:
        GCS_MAVLINK_send_heartbeat();
        break;

    case MSG_RAW_IMU:
        GCS_MAVLINK_send_raw_imu();
        break;

    case MSG_SCALED_IMU:
        GCS_MAVLINK_send_scaled_imu(0,mavlink_msg_scaled_imu_send);
        break;

    case MSG_SCALED_IMU2:
        GCS_MAVLINK_send_scaled_imu(1, mavlink_msg_scaled_imu2_send);
        break;

    case MSG_SCALED_IMU3:
        GCS_MAVLINK_send_scaled_imu(2, mavlink_msg_scaled_imu3_send);
        break;

    case MSG_SCALED_PRESSURE:
        GCS_MAVLINK_send_scaled_pressure_instance(0, mavlink_msg_scaled_pressure_send);
        break;

    case MSG_SCALED_PRESSURE2:
        GCS_MAVLINK_send_scaled_pressure_instance(1, mavlink_msg_scaled_pressure2_send);
        break;

    case MSG_SCALED_PRESSURE3:
        GCS_MAVLINK_send_scaled_pressure_instance(2, mavlink_msg_scaled_pressure3_send);
        break;

    case MSG_LOCAL_POSITION:
        GCS_MAVLINK_send_local_position();
        break;

    case MSG_OPTICAL_FLOW:
        GCS_MAVLINK_send_opticalflow();
        break;

    case MSG_LOCATION:
        GCS_MAVLINK_send_global_position_int();
        break;

    case MSG_RC_CHANNELS:
        GCS_MAVLINK_send_rc_channels();
        break;

    case MSG_RC_CHANNELS_RAW:

        //send_rc_channels_raw();
        break;

    case MSG_SERVO_OUTPUT_RAW:

        break;

    case MSG_DISTANCE_SENSOR:
        GCS_MAVLINK_send_distance_sensor();
        break;

    case MSG_EXTENDED_SYS_STATE :
        mavlink_msg_extended_sys_state_send(chan,0,copter_sys.landed_state);
        break;

    case MSG_BATTERY_STATUS:
        GCS_MAVLINK_send_battery_status();
        break;

    default:

            break;
        }

    return ret;
}
/*****************************Send MAVLink message END***************************************************/


/**************************Receive messages for non-specific vehicles*************************************/
uint32_t sysid_myggcs_last_seen_time_ms(){
    return _sysid_mygcs_last_seen_time_ms;
}
/*接收心跳消息*/
void handle_heartbeat(mavlink_message_t msg)
{
    // if the heartbeat is from our GCS then we don't failsafe for
    //if (msg.sysid == get_my_gcs_sysid())
    {
        _sysid_mygcs_last_seen_time_ms = rt_tick_get();
    }
}

void handle_command_ack(mavlink_message_t msg)
{
#if HAL_INS_ACCELCAL_ENABLED
    mavlink_command_ack_t packet;
    mavlink_msg_command_ack_decode(&msg, &packet);
#endif
}
void handle_setup_signing(mavlink_message_t msg)
{
    // decode
    mavlink_setup_signing_t packet;
    mavlink_msg_setup_signing_decode(&msg, &packet);
}

uint32_t bbb = 112;

// return a MAVLink parameter type given a AP_Param type
uint16_t mavlink_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
        return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
        return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
        return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}

float param[8][2]={     {2,6},      //RCMAP_PITCH
                        {1,6},
                        {3,6},
                        {4,6},
                        {2687498,6},
                        {0,6},
                        {0,6},
                        {0,6}
};//默认的串口协议
mavlink_param_value_t _var_info[50];

uint16_t return_number_of_arguments()
{
    for (int i = 0; i < 50; ++i) {

        if (_var_info[i].param_type == 0) {
            return i;
        }
    }
    return 0;
}



void GCS_MAVLINK_queued_param_send()
{
    char param_name0[ ] = "RCMAP_PITCH";
    for(int i=0; i<sizeof(param_name0)-1; i++)
    {
        _var_info[0].param_id[i] = param_name0[i];
    }
    char param_name1[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name1)-1; i++)
    {
        _var_info[1].param_id[i] = param_name1[i];
    }
    char param_name2[ ] = "RCMAP_THROTTLE";
    for(int i=0; i<sizeof(param_name2)-1; i++)
    {
        _var_info[2].param_id[i] = param_name2[i];
    }
    char param_name3[ ] = "RCMAP_YAW";
    for(int i=0; i<sizeof(param_name3)-1; i++)
    {
        _var_info[3].param_id[i] = param_name3[i];
    }
    char param_name4[ ] = "INS_ACC2_ID";
    for(int i=0; i<sizeof(param_name4)-1; i++)
    {
        _var_info[4].param_id[i] = param_name4[i];
    }
    char param_name5[ ] = "FLTMODE1";
    for(int i=0; i<sizeof(param_name5)-1; i++)
    {
        _var_info[5].param_id[i] = param_name5[i];
    }
    char param_name6[ ] = "FLTMODE2";
    for(int i=0; i<sizeof(param_name6)-1; i++)
    {
        _var_info[6].param_id[i] = param_name6[i];
    }
    char param_name7[ ] = "FLTMODE3";
    for(int i=0; i<sizeof(param_name7)-1; i++)
    {
        _var_info[7].param_id[i] = param_name7[i];
    }
    char param_name8[ ] = "FLTMODE4";
    for(int i=0; i<sizeof(param_name8)-1; i++)
    {
        _var_info[8].param_id[i] = param_name8[i];
    }
    char param_name9[ ] = "FLTMODE5";
    for(int i=0; i<sizeof(param_name9)-1; i++)
    {
        _var_info[9].param_id[i] = param_name9[i];
    }
    char param_name10[ ] = "FLTMODE6";
    for(int i=0; i<sizeof(param_name10)-1; i++)
    {
        _var_info[10].param_id[i] = param_name10[i];
    }
    char param_name11[ ] = "RC1_MIN";
    for(int i=0; i<sizeof(param_name11)-1; i++)
    {
        _var_info[11].param_id[i] = param_name11[i];
    }
    char param_name12[ ] = "RC1_MAX";
    for(int i=0; i<sizeof(param_name12)-1; i++)
    {
        _var_info[12].param_id[i] = param_name12[i];
    }
    char param_name13[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name13)-1; i++)
    {
        _var_info[13].param_id[i] = param_name13[i];
    }
    char param_name14[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name14)-1; i++)
    {
        _var_info[14].param_id[i] = param_name14[i];
    }
    char param_name15[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name15)-1; i++)
    {
        _var_info[15].param_id[i] = param_name15[i];
    }
    char param_name16[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name16)-1; i++)
    {
        _var_info[16].param_id[i] = param_name16[i];
    }
    char param_name17[ ] = "RCMAP_ROLL";
    for(int i=0; i<sizeof(param_name17)-1; i++)
    {
        _var_info[17].param_id[i] = param_name17[i];
    }


    for (int i = 0; i < 8; ++i) {
        _var_info[i].param_value = param[i][0];
    }
    for (int i = 0; i < 8; ++i) {
        _var_info[i].param_type = param[i][1];
    }



    rt_kprintf("_var_info:%s\r\n",_var_info[1].param_id);
    rt_kprintf("_var_info:%d\r\n",(int)return_number_of_arguments());

    int num = return_number_of_arguments();
    for (int i = 0;  i < num; ++ i) {
        mavlink_msg_param_value_send(
            0,
            _var_info[i].param_id,
            _var_info[i].param_value,
            _var_info[i].param_type,
            i+1,
            i);
    }



//
//    char param_namer[]="RCMAP_PITCH";
//    mavlink_msg_param_value_send(
//        0,
//        param_namer,
//        2,
//        5,
//        1,
//        0);
//    char param_name1[]="RCMAP_ROLL";
//    mavlink_msg_param_value_send(
//        0,
//        param_name1,
//        1,
//        6,
//        2,
//        1);
//    char param_name2[]="RCMAP_THROTTLE";
//    mavlink_msg_param_value_send(
//        0,
//        param_name2,
//        3,
//        5,
//        3,
//        2);
//    char param_name3[]="RCMAP_YAW";
//    mavlink_msg_param_value_send(
//        0,
//        param_name3,
//        4,
//        6,
//        4,
//        3);
//
//    char param_name4[]="FLTMODE_CH";
//    mavlink_msg_param_value_send(
//        0,
//        param_name4,
//        5,
//        6,
//        5,
//        4);
//
//    char param_name5[]="INS_ACC2_ID";
//    mavlink_msg_param_value_send(
//        0,
//        param_name5,
//        2687498,
//        6,
//        6,
//        5);
//
//    char param_name6[]="FLTMODE1";
//    mavlink_msg_param_value_send(
//        0,
//        param_name6,
//        0,
//        6,
//        7,
//        6);
//    char param_name7[]="FLTMODE2";
//    mavlink_msg_param_value_send(
//        0,
//        param_name7,
//        0,
//        6,
//        8,
//        7);
//    char param_name8[]="FLTMODE3";
//    mavlink_msg_param_value_send(
//        0,
//        param_name8,
//        0,
//        6,
//        9,
//        8);
//    char param_name9[]="FLTMODE4";
//    mavlink_msg_param_value_send(
//        0,
//        param_name9,
//        0,
//        6,
//        10,
//        9);
//    char param_name10[]="FLTMODE5";
//    mavlink_msg_param_value_send(
//        0,
//        param_name10,
//        0,
//        6,
//        11,
//        10);
//    char param_name11[]="FLTMODE6";
//    mavlink_msg_param_value_send(
//        0,
//        param_name11,
//        0,
//        6,
//        12,
//        11);
//    char param_name12[]="RC1_MIN";
//    mavlink_msg_param_value_send(
//        0,
//        param_name12,
//        1000,
//        6,
//        13,
//        13-1);
//    char param_name13[]="RC1_MAX";
//    mavlink_msg_param_value_send(
//        0,
//        param_name13,
//        2000,
//        6,
//        14,
//        14-1);
//    char param_name14[]="RC6_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name14,
//        0,
//        6,
//        15,
//        15-1);
//    char param_name15[]="RC1_TRIM";
//    mavlink_msg_param_value_send(
//        0,
//        param_name15,
//        1500,
//        6,
//        16,
//        16-1);
//    char param_name16[]="RC1_REV";
//    mavlink_msg_param_value_send(
//        0,
//        param_name16,
//        0,
//        6,
//        17,
//        17-1);
//
//    char param_name17[]="RC7_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name17,
//        0,
//        6,
//        18,
//        18-1);
//    char param_name18[]="RC8_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name18,
//        0,
//        6,
//        19,
//        19-1);
//    char param_name19[]="RC9_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name19,
//        0,
//        6,
//        20,
//        20-1);
//    char param_name20[]="RC10_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name20,
//        0,
//        6,
//        21,
//        21-1);
//    char param_name21[]="RC11_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name21,
//        0,
//        6,
//        22,
//        22-1);
//
//    char param_name22[]="RC12_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name22,
//        0,
//        6,
//        23,
//        23-1);
//    char param_name23[]="RC13_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name23,
//        0,
//        6,
//        24,
//        24-1);
//    char param_name24[]="RC14_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name24,
//        0,
//        6,
//        25,
//        25-1);
//    char param_name25[]="RC15_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name25,
//        0,
//        6,
//        26,
//        26-1);
//    char param_name26[]="RC16_OPTION";
//    mavlink_msg_param_value_send(
//        0,
//        param_name26,
//        0,
//        6,
//        27,
//        27-1);
}
/*参数类消息的接收处理*/
void handle_common_param_message(mavlink_message_t msg)
{
    // decode
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:  //未使用
        {
            mavlink_param_request_read_t packet;
            mavlink_msg_param_request_read_decode(&msg, &packet);
            rt_kprintf("packet20:%d\r\n",(rt_uint16_t)packet.param_index);

            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:  //QGC上位机请求参数
        {
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(&msg, &packet);
            rt_kprintf("packet21:%d\r\n",(rt_uint16_t)packet.target_component);
            if (streamRates[10]==0) {
                streamRates[10]=1;  //用于上位机请求参数的标志位
            }
            //GCS_MAVLINK_queued_param_send();
            break;
        }

    case MAVLINK_MSG_ID_PARAM_SET:  //上位机设置参数
        {
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(&msg, &packet);
            rt_kprintf("packet23:%s\r\n",packet.param_id);
            bbb = packet.param_value;
            if (!strcmp(packet.param_id,"FLTMODE1")) {
                mavlink_msg_param_value_send(
                    0,
                    packet.param_id,
                    bbb,
                    packet.param_type,
                    1,
                    0);
            }
            break;
        }
    }
}

void handle_set_gps_global_origin(mavlink_message_t msg)
{
    // decode
}
void handle_device_op_read(mavlink_message_t msg)
{
    // decode
}
void handle_device_op_write(mavlink_message_t msg)
{
    // decode
}
void handle_timesync(mavlink_message_t msg)
{
    // decode
}

void handle_file_transfer_protocol(mavlink_message_t msg)
{
    // decode
}

//设置飞行模式
MAV_RESULT _set_mode_common(const MAV_MODE _base_mode, const uint32_t _custom_mode)
{
    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
    if ((_base_mode) & 1) {
        if (!set_flight_mode(_custom_mode,GCS_COMMAND))
        {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }

    if (_base_mode == (MAV_MODE)128) {

        // set the safety switch position. Must be in a command by itself
        if (_custom_mode == 0) {
            return MAV_RESULT_ACCEPTED;
        }

        if (_custom_mode == 1) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_DENIED;
    }

    // Command is invalid (is supported but has invalid parameters)
    return MAV_RESULT_DENIED;
}

//接收处理设置飞行模式的消息
void handle_set_mode(mavlink_message_t msg)
{
    // decode
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(&msg, &packet);

    const MAV_MODE _base_mode = (MAV_MODE)packet.base_mode;
    const uint32_t _custom_mode = packet.custom_mode;

    const MAV_RESULT result = _set_mode_common(_base_mode, _custom_mode);

    //if (HAVE_PAYLOAD_SPACE(chan, COMMAND_ACK))//发送应答消息
    {
        mavlink_msg_command_ack_send(chan, MAVLINK_MSG_ID_SET_MODE, result,
                                     0, 0,
                                     msg.sysid,
                                     msg.compid);
    }
}

void handle_send_autopilot_version(mavlink_message_t msg)
{
    // decode
}
void handle_common_mission_message(mavlink_message_t msg)
{
    // decode
}

MAV_RESULT handle_command_set_message_interval(const mavlink_command_long_t packet)
{
    //uint32_t msg_id = (uint32_t)packet.param1;
    int32_t interval_us = (int32_t)packet.param2;
    uint16_t interval_ms;
    if (interval_us == 0) {
        // zero is "reset to default rate"
        //if (!get_default_interval_for_mavlink_message_id(msg_id, interval_ms))
        {
            return MAV_RESULT_FAILED;
        }
    } else if (interval_us == -1) {
        // minus-one is "stop sending"
        interval_ms = 0;
    } else if (interval_us < 1000) {
        // don't squash sub-ms times to zero
        interval_ms = 1;
    } else if (interval_us > 60000000) {
        interval_ms = 60000;
    } else {
        interval_ms = interval_us / 1000;
    }


    interval_us=interval_ms;
    //if (set_mavlink_message_id_interval(msg_id, interval_ms))
    {
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

/*接收处理设置模式的命令，并返回处理结果*/
MAV_RESULT handle_command_do_set_mode(const mavlink_command_long_t packet)
{
    const MAV_MODE _base_mode = (MAV_MODE)packet.param1;
    const uint32_t _custom_mode = (uint32_t)packet.param2;

    return _set_mode_common(_base_mode, _custom_mode);
}

/*接收处理解锁上锁的命令，并返回处理结果*/
MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_long_t packet)
{
    /*解锁命令*/
    if ((int)packet.param1 == 1) {
        if (cmd_unlock_arming()) {

            motor.state = motor_idle;  //电机怠速
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    /*上锁命令*/
    if (is_zero(packet.param1))  {
        if (1) {

            motor.state = motor_off;  //关闭所有电机
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
    }

    return MAV_RESULT_UNSUPPORTED;
}


/*接收处理飞控传感器校准的命令，并返回处理结果*/
MAV_RESULT handle_command_preflight_calibration(const mavlink_command_long_t packet)
{
    MAV_RESULT ret = MAV_RESULT_UNSUPPORTED;
    if ((int)packet.param1==1) {
        return MAV_RESULT_ACCEPTED;
    }

    if ((int)packet.param3==1) {

    }

    return ret;
}


uint16_t set_servo_channel=0,set_servo_pwm=0;

uint16_t get_set_servo_channel()
{
    return set_servo_channel;
}

uint16_t get_set_servo_pwm()
{
    return set_servo_pwm;
}
/*接收处理设置pwm通道输出值的命令，并返回处理结果*/
MAV_RESULT handle_set_servo_message(const mavlink_command_long_t packet)
{
    //MAV_RESULT ret = MAV_RESULT_UNSUPPORTED;
    if (packet.param1<3) {
        return MAV_RESULT_UNSUPPORTED;
    }
    if (packet.param2<3) {
        return MAV_RESULT_UNSUPPORTED;
    }

    set_servo_channel = (uint16_t)packet.param1;
    set_servo_pwm =  (uint16_t)packet.param2;

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t packet)
{
    if (motor.state != motor_off) {
        // refuse reboot when armed
        return MAV_RESULT_FAILED;
    }

    if (!(is_zero(packet.param1-1.0f) || is_zero(packet.param1-3.0f) )) {
        // param1 must be 1 or 3 - 1 being reboot, 3 being reboot-to-bootloader
        return MAV_RESULT_UNSUPPORTED;
    }
    // send ack before we reboot
    mavlink_msg_command_ack_send(0, packet.command, MAV_RESULT_ACCEPTED,
                                 0, 0, 0, 0);
    rt_thread_mdelay(100);
    rt_hw_cpu_reset();
    return MAV_RESULT_ACCEPTED;
}
/*接收各种命令,最后发送应答消息*/
void handle_command_long(mavlink_message_t msg)
{
    // decode packet
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(&msg, &packet);

    MAV_RESULT result = MAV_RESULT_FAILED;

     switch (packet.command) {

 #if HAL_INS_ACCELCAL_ENABLED
     case MAV_CMD_ACCELCAL_VEHICLE_POS:
         result = handle_command_accelcal_vehicle_pos(packet);
         break;
 #endif

     case MAV_CMD_DO_SET_MODE:
         result = handle_command_do_set_mode(packet);
         break;

 //    case MAV_CMD_DO_SEND_BANNER:
 //        //result = handle_command_do_send_banner(packet);
 //        break;

     case MAV_CMD_DO_SET_HOME:
         //result = handle_command_do_set_home(packet);
         break;

     case MAV_CMD_DO_FENCE_ENABLE:
         //result = handle_command_do_fence_enable(packet);
         break;

     case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
         result = handle_preflight_reboot(packet);
         break;

 #if HAL_HIGH_LATENCY2_ENABLED
     case MAV_CMD_CONTROL_HIGH_LATENCY:
         result = handle_control_high_latency(packet);
         break;
 #endif // HAL_HIGH_LATENCY2_ENABLED

 //    case MAV_CMD_DO_CANCEL_MAG_CAL: {
 //        //result = handle_command_mag_cal(packet);
 //        break;
 //    }

     case MAV_CMD_START_RX_PAIR:
         //result = handle_rc_bind(packet);
         break;

     case MAV_CMD_DO_DIGICAM_CONFIGURE:
     case MAV_CMD_DO_DIGICAM_CONTROL:
     case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
         //result = handle_command_camera(packet);
         break;

     case MAV_CMD_DO_GRIPPER:
         //result = handle_command_do_gripper(packet);
         break;

 #if HAL_SPRAYER_ENABLED
     case MAV_CMD_DO_SPRAYER:
         result = handle_command_do_sprayer(packet);
         break;
 #endif

     case MAV_CMD_DO_MOUNT_CONFIGURE:
     case MAV_CMD_DO_MOUNT_CONTROL:
     case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
         //result = handle_command_mount(packet);
         break;

     case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
         //result = handle_command_request_autopilot_capabilities(packet);
         break;
     }

     case MAV_CMD_DO_SET_ROI_SYSID:
        // return handle_command_do_set_roi_sysid(packet);

     case MAV_CMD_DO_SET_ROI_LOCATION:
     case MAV_CMD_DO_SET_ROI:
         //result = handle_command_do_set_roi(packet);
         break;

     case MAV_CMD_PREFLIGHT_CALIBRATION:
         result = handle_command_preflight_calibration(packet);
         break;


     case MAV_CMD_DO_SET_MISSION_CURRENT:
         //result = handle_command_do_set_mission_current(packet);
         break;

 //    case MAV_CMD_BATTERY_RESET:
 //        //result = handle_command_battery_reset(packet);
 //        break;

 #if HAL_ADSB_ENABLED
     case MAV_CMD_DO_ADSB_OUT_IDENT:
         if ((AP::ADSB() != nullptr) && AP::ADSB()->ident_start()) {
             result = MAV_RESULT_ACCEPTED;
         }
         else {
             result = MAV_RESULT_FAILED;
         }
         break;
 #endif

     case MAV_CMD_PREFLIGHT_UAVCAN:
         //result = handle_command_preflight_can(packet);
         break;

     case MAV_CMD_RUN_PREARM_CHECKS:
         //result = handle_command_run_prearm_checks(packet);
         break;

 //    case MAV_CMD_FLASH_BOOTLOADER:
 //        //result = handle_command_flash_bootloader(packet);
 //        break;

     case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: {
         //result = handle_command_preflight_set_sensor_offsets(packet);
         break;
     }

     case MAV_CMD_GET_HOME_POSITION:
         //result = handle_command_get_home_position(packet);
         break;

 //    case MAV_CMD_DEBUG_TRAP:
 //        //result = handle_command_debug_trap(packet);
 //        break;

 //    case MAV_CMD_SET_EKF_SOURCE_SET:
 //        //result = handle_command_set_ekf_source_set(packet);
 //        break;

     case MAV_CMD_PREFLIGHT_STORAGE:
 //        if (is_equal(packet.param1, 2.0f)) {
 //            AP_Param::erase_all();
 //            send_text(MAV_SEVERITY_WARNING, "All parameters reset, reboot board");
 //            result= MAV_RESULT_ACCEPTED;
 //        }
         break;

 //    case MAV_CMD_DO_AUX_FUNCTION:
 //        //result = handle_command_do_aux_function(packet);
 //        break;

     case MAV_CMD_SET_MESSAGE_INTERVAL:
         result = handle_command_set_message_interval(packet);
         break;

     case MAV_CMD_GET_MESSAGE_INTERVAL:
         //result = handle_command_get_message_interval(packet);
         break;

     case MAV_CMD_REQUEST_MESSAGE:
         //result = handle_command_request_message(packet);
         break;

     case MAV_CMD_DO_SET_SERVO:
         result = handle_set_servo_message(packet);
     case MAV_CMD_DO_REPEAT_SERVO:

     case MAV_CMD_DO_SET_RELAY:

     case MAV_CMD_DO_REPEAT_RELAY:
         //result = handle_servorelay_message(packet);
         break;

     case MAV_CMD_DO_FLIGHTTERMINATION:
         //result = handle_flight_termination(packet);
         break;

     case MAV_CMD_COMPONENT_ARM_DISARM:
         result = handle_command_component_arm_disarm(packet);
         break;

     case MAV_CMD_FIXED_MAG_CAL_YAW:
         //result = handle_fixed_mag_cal_yaw(packet);
         break;

     default:
         result = MAV_RESULT_UNSUPPORTED;
         break;
     }
//    rt_kprintf("handle_common_message");
//    rt_kprintf("handle %u  %u\n", packet.command,(int)result);
//    rt_kprintf("param1 %u  %u\n", (int)packet.param1,(uint16_t)packet.param2);
//    rt_kprintf("param3 %u  %u\n", (int)packet.param3,(int)packet.param4);
//    rt_kprintf("param5 %u  %u\n", (int)packet.param5,(int)packet.param6);
//    rt_kprintf("param7 %u  %u\n", (int)packet.param7,(int)packet.param7);
    // send ACK or NAK
    mavlink_msg_command_ack_send(chan, packet.command, result,
                                 0, 0,
                                 msg.sysid,
                                 msg.compid);
}
void handle_command_int(mavlink_message_t msg)
{
    // decode
}
void handle_fence_message(mavlink_message_t msg)
{
    // decode
}
void handle_param_value(mavlink_message_t msg)
{
    // decode
}
void handle_serial_control(mavlink_message_t msg)
{
    // decode
}
void handle_statustext(mavlink_message_t msg)//接收文本消息
{
    // decode
    mavlink_statustext_t packet;
    mavlink_msg_statustext_decode(&msg, &packet);
    char text[71] = { 'G','C','S',':'};
    uint8_t offset = strlen(text);
    memcpy(&text[offset], packet.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
}

/*请求数据流的消息处理*/
void handle_request_data_stream(mavlink_message_t msg)
{
    // decode
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(&msg, &packet);

    int16_t freq = 0;                     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                         // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;   // start sending
    else
        return;

    // if stream_id is still NUM_STREAMS at the end of this switch
    // block then either we set stream rates for all streams, or we
    // were asked to set the streamrate for an unrecognised stream
    enum streams stream_id = NUM_STREAMS;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        for (uint8_t i=0; i<NUM_STREAMS; i++) {
            if (i == STREAM_PARAMS) {
                // don't touch parameter streaming rate; it is
                // considered "internal".
                continue;
            }
            streamRates[i] = freq;
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        stream_id = STREAM_RAW_SENSORS;
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        stream_id = STREAM_EXTENDED_STATUS;
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        stream_id = STREAM_RC_CHANNELS;
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        stream_id = STREAM_RAW_CONTROLLER;
        break;
    case MAV_DATA_STREAM_POSITION:
        stream_id = STREAM_POSITION;
        break;
    case MAV_DATA_STREAM_EXTRA1:
        stream_id = STREAM_EXTRA1;
        break;
    case MAV_DATA_STREAM_EXTRA2:
        stream_id = STREAM_EXTRA2;
        break;
    case MAV_DATA_STREAM_EXTRA3:
        stream_id = STREAM_EXTRA3;
        break;
    }

    if (stream_id == NUM_STREAMS) {
        // asked to set rate on unknown stream (or all were set already)
        return;
    }

    streamRates[stream_id] = freq;

}
void handle_data_packet(mavlink_message_t msg)
{
    // decode
}
void handle_vision_position_delta(mavlink_message_t msg)
{
    // decode
}

/*视觉位置估计消息的接收*/
void handle_vision_position_estimate(mavlink_message_t msg)
{
    mavlink_vision_position_estimate_t m;
    mavlink_msg_vision_position_estimate_decode(&msg, &m);
    _vision_position_estimate = m;
}
void handle_global_vision_position_estimate(mavlink_message_t msg)
{
    // decode
}

mavlink_vision_position_estimate_t get_vision_position_estimate(void)
{
    return _vision_position_estimate;
} //返回当前位置

/*光学动作捕捉位置估计消息的接收*/
void handle_vicon_position_estimate(mavlink_message_t msg)
{
    // decode
}
void handle_odometry(mavlink_message_t msg)
{
    // decode
}
void handle_att_pos_mocap(mavlink_message_t msg)
{
    // decode
}

/*视觉速度估计消息的接收*/
void handle_vision_speed_estimate(mavlink_message_t msg)
{
    mavlink_vision_speed_estimate_t m;
    mavlink_msg_vision_speed_estimate_decode(&msg, &m);
}
void handle_system_time_message(mavlink_message_t msg)
{
    // decode
}
void handle_rc_channels_override(mavlink_message_t msg)
{
    // decode
}
void handle_optical_flow(mavlink_message_t msg)
{
    // decode
}
void handle_distance_sensor(mavlink_message_t msg)
{
    // decode
}
void handle_obstacle_distance(mavlink_message_t msg)
{
    // decode
}
void handle_obstacle_distance_3d(mavlink_message_t msg)
{
    // decode
}
void handle_osd_param_config(mavlink_message_t msg)
{
    // decode
}
void handle_adsb_message(mavlink_message_t msg)
{
    // decode
}
void handle_landing_target(mavlink_message_t msg)
{
    // decode
}void handle_named_value(mavlink_message_t msg)
{
    // decode
}
void handle_can_frame(mavlink_message_t msg)
{
    // decode
}


//各种mavlink消息和命令的接收
void GCS_MAVLINK_handle_common_message(mavlink_message_t msg)//handle messages which don't require vehicle specific data
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        handle_heartbeat(msg);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        handle_command_ack(msg);
        break;
    }

    case MAVLINK_MSG_ID_SETUP_SIGNING:
        handle_setup_signing(msg);
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_common_param_message(msg);
        break;

    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        handle_set_gps_global_origin(msg);
        break;

//    case MAVLINK_MSG_ID_DEVICE_OP_READ:
//        handle_device_op_read(msg);
//        break;
//    case MAVLINK_MSG_ID_DEVICE_OP_WRITE:
//        handle_device_op_write(msg);
//        break;
    case MAVLINK_MSG_ID_TIMESYNC:
        handle_timesync(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
//    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
//        AP::logger().handle_mavlink_msg(*this, msg);
//        break;

    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
        handle_file_transfer_protocol(msg);
        break;

//    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
//    case MAVLINK_MSG_ID_GOPRO_HEARTBEAT: // heartbeat from a GoPro in Solo gimbal
//        {
//            AP_Camera *camera = AP::camera();
//            if (camera == nullptr) {
//                return;
//            }
//            camera->handle_message(chan, msg);
//        }
//        break;

    case MAVLINK_MSG_ID_SET_MODE:
        handle_set_mode(msg);
        break;

//    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
//        handle_send_autopilot_version(msg);
//        break;

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        handle_common_mission_message(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_INT:
        handle_command_int(msg);
        break;

//    case MAVLINK_MSG_ID_FENCE_POINT:
//    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
//        handle_fence_message(msg);
//        break;

#if HAL_MOUNT_ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE: // deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
        send_received_message_deprecation_warning("MOUNT_CONFIGURE");
        handle_mount_message(msg);
        break;
    case MAVLINK_MSG_ID_MOUNT_CONTROL: // deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
        send_received_message_deprecation_warning("MOUNT_CONTROL");
        handle_mount_message(msg);
        break;
    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        handle_mount_message(msg);
        break;
#endif

    case MAVLINK_MSG_ID_PARAM_VALUE:
        handle_param_value(msg);
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg);
        break;

    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
    case MAVLINK_MSG_ID_GPS_INPUT:
    case MAVLINK_MSG_ID_HIL_GPS:
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:

    case MAVLINK_MSG_ID_STATUSTEXT:
        handle_statustext(msg);
        break;

//    case MAVLINK_MSG_ID_LED_CONTROL:
//        // send message to Notify
//        //AP_Notify::handle_led_control(msg);
//        break;

    case MAVLINK_MSG_ID_PLAY_TUNE:


#if HAL_RALLY_ENABLED
    case MAVLINK_MSG_ID_RALLY_POINT:
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        handle_common_rally_message(msg);
        break;
#endif

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        // only pass if override is not selected
        //if (!(_port->get_options() & _port->OPTION_NOSTREAMOVERRIDE))
        {
            handle_request_data_stream(msg);
        }
        break;

//    case MAVLINK_MSG_ID_DATA96:
//        handle_data_packet(msg);
//        break;

//    case MAVLINK_MSG_ID_VISION_POSITION_DELTA:
//        handle_vision_position_delta(msg);
//        break;

    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        handle_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
        handle_global_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
        handle_vicon_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_ODOMETRY:
        handle_odometry(msg);
        break;

    case MAVLINK_MSG_ID_ATT_POS_MOCAP:
        handle_att_pos_mocap(msg);
        break;

    case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
        handle_vision_speed_estimate(msg);
        break;

    case MAVLINK_MSG_ID_SYSTEM_TIME:
        handle_system_time_message(msg);
        break;

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        handle_rc_channels_override(msg);
        break;

#if AP_OPTICALFLOW_ENABLED
    case MAVLINK_MSG_ID_OPTICAL_FLOW:
        handle_optical_flow(msg);
        break;
#endif

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        handle_distance_sensor(msg);
        break;

    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
        handle_obstacle_distance(msg);
        break;

//    case MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D:
//        handle_obstacle_distance_3d(msg);
//        break;

//    case MAVLINK_MSG_ID_OSD_PARAM_CONFIG:
//    case MAVLINK_MSG_ID_OSD_PARAM_SHOW_CONFIG:
//        handle_osd_param_config(msg);
//        break;

    case MAVLINK_MSG_ID_ADSB_VEHICLE:
//    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
//    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_DYNAMIC:
//    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
//    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CONTROL:
//        handle_adsb_message(msg);
//        break;

    case MAVLINK_MSG_ID_LANDING_TARGET:
        handle_landing_target(msg);
        break;

    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        handle_named_value(msg);
        break;

//    case MAVLINK_MSG_ID_CAN_FRAME:
//    case MAVLINK_MSG_ID_CANFD_FRAME:
//        handle_can_frame(msg);
//        break;

//    case MAVLINK_MSG_ID_CAN_FILTER_MODIFY:
//#if HAL_CANMANAGER_ENABLED
//        AP::can().handle_can_filter_modify(msg);
//#endif
//        break;

#if AP_OPENDRONEID_ENABLED
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE:
        AP::opendroneid().handle_msg(chan, msg);
        break;
#endif

#if AP_SIGNED_FIRMWARE
    case MAVLINK_MSG_ID_SECURE_COMMAND:
    case MAVLINK_MSG_ID_SECURE_COMMAND_REPLY:
        AP_CheckFirmware::handle_msg(chan, msg);
        break;
#endif
    }

}
/**************************Receive messages for non-specific vehicles END*************************************/
