/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-25     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_SERIALPORT_DL_SERIALPORT_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_SERIALPORT_DL_SERIALPORT_H_

#include "stdbool.h"

enum SerialProtocol {
        SerialProtocol_None = -1,
        SerialProtocol_Console = 0,
        SerialProtocol_MAVLink = 1,
        SerialProtocol_Tof = 2,
        SerialProtocol_Vision = 3,
        SerialProtocol_GPS = 4,
        SerialProtocol_Opt = 5,
        SerialProtocol_Openmv = 6,
        SerialProtocol_Opt2 = 7,
        SerialProtocol_KS103 = 8,
        SerialProtocol_US100 = 9,
        SerialProtocol_ROS = 10,
        SerialProtocol_Sbus =11,
        SerialProtocol_NumProtocols
    };

//enum SerialProtocol {
//    SerialProtocol_None = -1,
//    SerialProtocol_Console = 0,                  // unused
//    SerialProtocol_MAVLink = 1,
//    SerialProtocol_MAVLink2 = 2,                 // do not use - use MAVLink and provide instance of 1
//    SerialProtocol_GPS = 5,
//    SerialProtocol_GPS2 = 6,                     // do not use - use GPS and provide instance of 1
//    SerialProtocol_Rangefinder = 9,
//    SerialProtocol_Lidar360 = 11,                // Lightware SF40C, TeraRanger Tower or RPLidarA2
//    SerialProtocol_Sbus1 = 15,
//    SerialProtocol_ESCTelemetry = 16,
//    SerialProtocol_OpticalFlow = 18,
//    SerialProtocol_RCIN = 23,
//    SerialProtocol_EFI = 24,                   // EFI serial protocol
//    SerialProtocol_Scripting = 28,
//    SerialProtocol_AirSpeed = 34,
//    SerialProtocol_ADSB = 35,
//    SerialProtocol_AHRS = 36,
//    SerialProtocol_MSP_DisplayPort = 42,
//    SerialProtocol_MAVLinkHL = 43,
//    SerialProtocol_NumProtocols                    // must be the last value
//};
rt_device_t serial_device_find(enum SerialProtocol protocol);
void set_serial_device_baud_rate(rt_device_t serial,rt_uint32_t buad_rate);


extern uint16_t uartProtocol[8][2];


enum external_sensor_id {
        Tof_id             = 1,
        OpticalFlow_id     = 2,
        T265_id            = 3,
        Openmv_id          = 4,
        US100_id           = 5,
        GPS_id             = 6,
        LaserRadar_id      = 7,
        external_sensor_Num_id
    };
bool is_use_external_sensor(enum external_sensor_id id);  //用来判断是否采用外部的某个传感器

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_SERIALPORT_DL_SERIALPORT_H_ */
