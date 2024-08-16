/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-07-19     李梦辉Q       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_W25QXX_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_W25QXX_H_
#include "rtthread.h"
#include <stdio.h>


//W25X系列/Q系列芯片列表
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16
//W25Q128 ID  0XEF17
#define W25Q80  0XEF13
#define W25Q16  0XEF14
#define W25Q32  0XEF15
#define W25Q64  0XEF16
#define W25Q128 0XEF17

extern rt_uint16_t W25QXX_TYPE;                 //定义W25QXX芯片型号


//////////////////////////////////////////////////////////////////////////////////
//指令表
#define W25X_WriteEnable        0x06
#define W25X_WriteDisable       0x04
#define W25X_ReadStatusReg      0x05
#define W25X_WriteStatusReg     0x01
#define W25X_ReadData           0x03
#define W25X_FastReadData       0x0B
#define W25X_FastReadDual       0x3B
#define W25X_PageProgram        0x02
#define W25X_BlockErase         0xD8
#define W25X_SectorErase        0x20
#define W25X_ChipErase          0xC7
#define W25X_PowerDown          0xB9
#define W25X_ReleasePowerDown   0xAB
#define W25X_DeviceID           0xAB
#define W25X_ManufactDeviceID   0x90
#define W25X_JedecDeviceID      0x9F


int w25qxx_init(void);
int w25qxx_spi_device_init(void);
int w25qxx_read_id(void);
rt_uint8_t W25QXX_ReadSR(void);
void W25QXX_Write_SR(rt_uint8_t sr);
void w25qxx_Read(rt_uint8_t* pBuffer,rt_uint32_t ReadAddr,rt_uint16_t NumByteToRead);
void w25qxx_write_page(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite);
void w25qxx_write_nocheck(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite);
void w25qxx_Write(rt_uint8_t* pBuffer,rt_uint32_t WriteAddr,rt_uint16_t NumByteToWrite);
void W25QXX_Write_Enable(void);
void W25QXX_Erase_Chip(void);
void W25QXX_Erase_Sector(rt_uint32_t Dst_Addr);
void W25QXX_Wait_Busy(void);
void W25QXX_Erase_Sector(rt_uint32_t Dst_Addr);




#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_DATESAVE_DL_W25QXX_H_ */
