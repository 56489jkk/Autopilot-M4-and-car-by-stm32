/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-20     CGY       the first version
 */
#ifndef APPLICATIONS_LOCO_CONFIG_H_
#define APPLICATIONS_LOCO_CONFIG_H_

#include <rtdbg.h>
#include <board.h>
#include <rtdevice.h>
#include "board_interface.h"
#include "AutoCopter.h"

#include <DL_SerialPort.h>
#include "DL_InertialSensor.h"
#include "DL_InertialSensor_icm20602.h"
#include "DL_InertialSensor_icm20689.h"
#include "DL_InertialSensor_icm42605.h"
#include "DL_Compass_hmc5883l.h"
#include "DL_BaroSensor.h"
#include "DL_Baro_spl06.h"
#include "DL_Baro_bmp388.h"
#include "DL_GPS_Sensor.h"
#include "DL_T265.h"

#include "DL_OpticalFlow.h"
#include "DL_RangeSensor.h"
#include "DL_TFmini.h"
#include "DL_ultrasonic.h"

#include "stdint.h"
#include "stdbool.h"
#include "usb_transfer.h"

#include "CL_WayNavigation.h"
#include "CL_PositionCtrl.h"
#include "CL_AttitudeCtrl.h"
#include "CL_DigitalFilter.h"
#include "CL_MotorsCopter.h"
#include "CL_NavigationKF.h"
#include "CL_RC_Channel.h"
#include "CL_Vector.h"
#include "CL_Math.h"
#include "CL_Matrix.h"
#include "CL_AHRS.h"
#include "CL_ADRC.h"
#include "CL_PID.h"
#include "CL_Notify.h"
#include "CL_MathCtrl.h"

//传感器的SPI驱动名称 一次从01至20     即能容纳20个spi通信的传感器
#define ICM20602_SPI_DEVICE_NAME        "spi01" //ICM20602
#define ICM42605_SPI_DEVICE_NAME        "spi02" //ICM42605
#define BMM150_SPI_DEVICE_NAME          "spi03" //BMM150
#define SPL06_SPI_DEVICE_NAME           "spi04" //SPL06
#define BMP388_SPI_DEVICE_NAME          "spi05" //BMP388
#define ICM20689_SPI_DEVICE_NAME        "spi06" //ICM20689

//flash、屏的SPI驱动名称 一次从21至30   即能容纳10个spi通信的芯片
#define W25QXX_SPI_DEVICE_NAME          "spi21" //W25QXX
#define FM25V_SPI_DEVICE_NAME           "spi22" //W25QXX
#define SD_CARD_SPI_DEVICE_NAME         "spi23" //SD卡
#define DISPLAY_SPI_DEVICE_NAME         "spi24" //显示屏

#define UART1_NAME       "uart1"    /* 串口设备名称 */
#define UART2_NAME       "uart2"    /* 串口设备名称 */
#define UART3_NAME       "uart3"    /* 串口设备名称 */
#define UART4_NAME       "uart4"    /* 串口设备名称 */
#define UART5_NAME       "uart5"    /* 串口设备名称 */
#define UART6_NAME       "uart6"    /* 串口设备名称 */
#define UART7_NAME       "uart7"    /* 串口设备名称 */
#define UART8_NAME       "uart8"    /* 串口设备名称 */




#endif /* APPLICATIONS_LOCO_CONFIG_H_ */
