/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-30     CGY       the first version
 */
#ifndef APPLICATIONS_INCLUDE_DL_ULTRASONIC_H_
#define APPLICATIONS_INCLUDE_DL_ULTRASONIC_H_


typedef struct {
    uint8_t cmd[30];      //对超声波发的命令 标志位等
    uint8_t name;         //哪一个超声波  0x01为ks103超声波
    uint8_t data[30];     //数据
    uint8_t dataLen;      //数据长度
    uint8_t health;       //是否能使用
    float distance;       //测距长度
    float last_distance;  //上次距离
    float div;            //距离的微分
    float temperature;    //温度
}Ultrason_data;           //超声波数据结构体

extern Ultrason_data ultr;   //超声波数据结构体

void ultra_start_work(void);
void ultrasonic_receive(uint8_t com_data);

#endif /* APPLICATIONS_INCLUDE_ULTRASONIC_H_ */
