/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-19     李梦辉Q       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_FM25VXX_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_FM25VXX_H_
#include "rtthread.h"
#include <stdio.h>


//FM25VX系列
//FM25V02  ID  0X2208
//FM25V05  ID  0X2300

#define FM25V02    0X2208          // 0000h to 7FFFh
#define FM25V05    0X2300          //


#define FM25_WREN            0x06   //使能写
#define FM25_WRDI            0x04   //失能写
#define FM25_RDSR            0x05   //读状态寄存器
#define FM25_WRSR            0x01   //写状态寄存器
#define FM25_READ            0x03   //读数据
#define FM25_FSTRD           0x0B   //快速读数据
#define FM25_WRITE           0x02   //写数据
#define FM25_RDID            0x9F   //读设备ID
#define FM25_DUMMY_BYTE      0xFF   //虚拟字节

int fm25vxx_read_id(void);
void fm25vxx_write(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite);
void fm25vxx_Read(rt_uint8_t* pBuffer,rt_uint32_t ReadAddr,rt_uint16_t NumByteToRead);


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_W25QXX_H_ */
