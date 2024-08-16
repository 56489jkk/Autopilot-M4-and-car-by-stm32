/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_BMP388_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_BMP388_H_


#define BMP_CHIP_ID         0x00

#define BMP_DATA_0          0x04
#define BMP_DATA_1          0x05
#define BMP_DATA_2          0x06
#define BMP_DATA_3          0x07
#define BMP_DATA_4          0x08
#define BMP_DATA_5          0x09

#define BMP_PWR_CTRL        0x1B
#define BMP_OSR             0x1C
#define BMP_ODR             0x1D
#define BMP_CONFIG          0x1F
#define BMP_CMD             0x7E

#define BMP_ADR             0x31

#define BMP_WHO_AM_I       (0x50)

struct bmp388_calib_param_t {
    int16_t t1;
    int16_t t2;
    int8_t  t3;
    int16_t p1;
    int16_t p2;
    int8_t  p3;
    int8_t  p4;
    int16_t p5;
    int16_t p6;
    int8_t  p7;
    int8_t  p8;
    int16_t p9;
    int8_t  p10;
    int8_t  p11;
};

struct bmp388_t {
    struct bmp388_calib_param_t calib_param;/**<calibration data*/
    uint8_t     chip_id; /**<chip id*/
    int32_t     i32rawPressure;
    int32_t     i32rawTemperature;
    int32_t     i32kP;
    int32_t     i32kT;
};

float bmp388_get_temperature(void);
float bmp388_get_pressure(void);



#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_BMP388_H_ */
