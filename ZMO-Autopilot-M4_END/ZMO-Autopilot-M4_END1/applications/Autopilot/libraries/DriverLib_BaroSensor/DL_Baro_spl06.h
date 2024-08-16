/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_SPL06_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_SPL06_H_
#include "loco_config.h"

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


struct spl0601_calib_param_t {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
};

struct spl0601_t {
    struct spl0601_calib_param_t calib_param;/**<calibration data*/
    uint8_t     chip_id; /**<chip id*/
    int32_t     i32rawPressure;
    int32_t     i32rawTemperature;
    int32_t     i32kP;
    int32_t     i32kT;
};

int spl06_read_id(void);

uint8_t spl06_reg_read(rt_uint8_t addr);

float spl0601_get_pressure ( void );
float spl0601_get_temperature ( void );


#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_BAROSENSOR_DL_BARO_SPL06_H_ */
