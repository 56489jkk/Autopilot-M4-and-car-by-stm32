/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-11     CGY       the first version
 */
#ifndef APPLICATIONS_USB_TRANSFER_H_
#define APPLICATIONS_USB_TRANSFER_H_
#include <board.h>
#include "CL_Vector.h"
#include <rtdbg.h>

#define MAX_DATA_SIZE 30

extern rt_device_t dev;

extern uint8_t ID;
extern int debug_flag[5];

extern float debug_fg[5];

extern Vector3i16 raw_magn;
//数据返回周期时间（单位ms）
#define  PERIOD_STATUS      15
#define  PERIOD_SENSOR      5
#define  PERIOD_RCDATA      50
#define  PERIOD_POWER       100
#define  PERIOD_MOTOR       10
#define  PERIOD_SENSOR2     10
#define  PERIOD_SPEED       50




#define PID_1_P     1
#define PID_1_I     2
#define PID_1_D     3
#define PID_2_P     4
#define PID_2_I     5
#define PID_2_D     6
#define PID_3_P     7
#define PID_3_I     8
#define PID_3_D     9
#define PID_4_P     10
#define PID_4_I     11
#define PID_4_D     12
#define PID_5_P     13
#define PID_5_I     14
#define PID_5_D     15
#define PID_6_P     16
#define PID_6_I     17
#define PID_6_D     18
#define PID_7_P     19
#define PID_7_I     20
#define PID_7_D     21
#define PID_8_P     22
#define PID_8_I     23
#define PID_8_D     24
#define PID_9_P     25
#define PID_9_I     26
#define PID_9_D     27
#define PID_10_P        28
#define PID_10_I        29
#define PID_10_D        30
#define PID_11_P        31
#define PID_11_I        32
#define PID_11_D        33
#define PID_12_P        34
#define PID_12_I        35
#define PID_12_D        36
#define PID_13_P        37
#define PID_13_I        38
#define PID_13_D        39
#define PID_14_P        40
#define PID_14_I        41
#define PID_14_D        42
#define PID_15_P        43
#define PID_15_I        44
#define PID_15_D        45
#define PID_16_P        46
#define PID_16_I        47
#define PID_16_D        48
#define PID_17_P        49
#define PID_17_I        50
#define PID_17_D        51
#define PID_18_P        52
#define PID_18_I        53
#define PID_18_D        54
typedef struct
{
    uint8_t dataLen;
    uint8_t data[MAX_DATA_SIZE];
}message;



void ZMO_PC_data_exchange(void);

void send_senser(rt_int16_t a_x,rt_int16_t a_y,rt_int16_t a_z,
                 rt_int16_t g_x,rt_int16_t g_y,rt_int16_t g_z,
                 rt_int16_t m_x,rt_int16_t m_y,rt_int16_t m_z);
void send_status(float angle_rol, float angle_pit, float angle_yaw,
                 rt_uint32_t alt, rt_uint8_t fly_model, rt_uint8_t armed);

void send_senser2(rt_int32_t bar_alt,rt_int32_t csb_alt, rt_int16_t sensertmp);

void send_rc_data(rt_uint16_t thr, rt_uint16_t yaw, rt_uint16_t rol,rt_uint16_t pit,
                  rt_uint16_t aux1,rt_uint16_t aux2,rt_uint16_t aux3,
                  rt_uint16_t aux4,rt_uint16_t aux5,rt_uint16_t aux6);
void send_power(rt_uint16_t votage, rt_uint16_t current);
void send_location(rt_uint8_t state,rt_uint8_t sat_num,rt_int32_t lon,rt_int32_t lat,float back_home_angle);
void send_speed(float x_s,float y_s,float z_s);
void send_moto_pwm(rt_uint16_t m_1,rt_uint16_t m_2,rt_uint16_t m_3,rt_uint16_t m_4,
                   rt_uint16_t m_5,rt_uint16_t m_6,rt_uint16_t m_7,rt_uint16_t m_8);
void send_dbug_data(float angle_rol, float angle_pit, float angle_yaw,
                 rt_uint32_t alt, rt_uint8_t fly_model, rt_uint8_t armed);
void send_string(const char *str);
void send_parame(rt_uint16_t num);
void get_parame_data(rt_uint16_t num,rt_int32_t data);
void send_cmd(rt_uint8_t dest, rt_uint8_t fun, rt_uint16_t cmd1, rt_uint16_t cmd2,
              rt_uint16_t cmd3, rt_uint16_t cmd4, rt_uint16_t cmd5);
void ParUsedToParList(void);
void Par_To_ParUsed(void);
void data_receive_parse(rt_uint8_t *data_buf,rt_uint8_t num);
void send_debug_data(float angle_rol, float angle_pit, float angle_yaw,
                 rt_uint32_t alt, rt_uint8_t fly_model, rt_uint8_t armed);

void data_receive_prepare(uint8_t data);


#endif /* APPLICATIONS_USB_TRANSFER_H_ */
