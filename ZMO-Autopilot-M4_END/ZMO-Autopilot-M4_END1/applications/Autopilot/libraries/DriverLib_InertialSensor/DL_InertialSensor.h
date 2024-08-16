/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_H_

#define MAX_IMU_NUM  3

#include "loco_config.h"
#include "stdbool.h"
#include "CL_Vector.h"
#include "CL_Math.h"
//imu数据状态
typedef struct
{
    //传感器的序列数，每成功获取到传感器的设备句柄就会自加一，其数值代表每个传感器的序号，按此序号就可以读取不同的传感器
    //此序号加一并不代表传感器数据是否正常，只能代表注册了多少个传感器。
    uint8_t instance;

    bool gyro_healthy[MAX_IMU_NUM];
    bool acce_healthy[MAX_IMU_NUM];
    bool magn_healthy[MAX_IMU_NUM];
    bool baro_healthy[MAX_IMU_NUM];
} IMU_state;

extern IMU_state imu;
#define Calibration_Gyro_Cycle         500.0f
//mpu6000初始化陀螺仪量程为FSR_2000DPS，即
#define GYRO_SCALE     0.06103515625f                      // 1/16.384f
#define ACC_1G_ADC     0.2392578125f                       // 1/(4096.0f/Gravity_Acce)
#define Calibration_Gyro_Cycle         500.0f
//mpu6000初始化陀螺仪量程为FSR_2000DPS，即
typedef struct
{
    Vector3i64 Sum;
    Vector3f   Zero;
    stdev_t  var[3];
    uint16_t cycleCount;
}gyroCalibration_t;

extern  gyroCalibration_t  gyroCalibration[MAX_IMU_NUM];

int get_gyro_instance(void);
void gyro_data_update(Vector3f *gyro,uint8_t instance);
void acce_data_update(Vector3f *acce,uint8_t instance);

//设置陀螺仪求和次数
void Set_Gyro_Sum_Cnt(uint16_t calibrationCyclesRequired,uint8_t instance);
bool gyroCalibration_cnt_init(uint8_t instance);
bool gyroIsCalibrationComplete(uint8_t instance);
bool Gyro_Sum_is_ok(uint8_t instance);
void Gyro_Calibration(Vector3i16 gyroADCsample,uint8_t instance);

/*从队列读取陀螺数据*/
bool sensorsReadGyro(Vector3f *gyro);
/*从队列读取加速计数据*/
bool sensorsReadAcce(Vector3f *acce);
Vector3f get_imu_sensorsGyro(void);      //为原始数据经第一次低通滤波后的角速度数据 deg/s
Vector3f get_imu_sensorsAcce(void);  //为原始数据经第一次低通滤波后的角速度数据 deg/s
Vector3f get_raw_imu_sensorsGyro(void);
void applySensorAlignment(int16_t * dest, int16_t * src, uint8_t rotation);

int get_gyro_queue_entry();
int get_acce_queue_entry();




#define Calibration_Acce_Cycle         500.0f

typedef struct
{
    Vector3i64 Sum;
    Vector3f   Zero;
    stdev_t  var[3];
    uint16_t cycleCount;
}acceCalibration_t;

extern  acceCalibration_t  acceCalibration[MAX_IMU_NUM];

//陀螺仪是否校准成功。当计数器递减到零时，函数返回true
bool acceIsCalibrationComplete(uint8_t instance);
void Acce_Calibration(Vector3i16 acceADCsample,uint8_t instance);
//设置陀螺仪求和次数
void Set_Acce_Sum_Cnt(uint16_t calibrationCyclesRequired,uint8_t instance);



#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_INERTIALSENSOR_DL_INERTIALSENSOR_H_ */







