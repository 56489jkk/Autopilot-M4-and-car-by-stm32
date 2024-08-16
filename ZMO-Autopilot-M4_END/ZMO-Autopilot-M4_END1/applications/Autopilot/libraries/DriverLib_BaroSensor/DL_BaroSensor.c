/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-29     CGY       the first version
 */
#include "loco_config.h"
#include "DL_BaroSensor.h"

BARO_state alt={0};
static float baroGroundPressure = 101325.0f; // 101325 pascal, 1 standard atmosphere
static uint32_t baroCalibrationTimeout = 0;
static float baroGroundAltitude = 0;
static bool baroCalibrationFinished = false;


float _baro_altitude;
/*读取气压计数据*/
float get_baro_sensorsAltitude(void)  //海拔高度开机时复位为0  单位cm
{
    return _baro_altitude;
}
/****************************************************************************************
*@brief   传感器的气压数据读取及处理
*@param[in]
*****************************************************************************************/
void Baro_Data_Update(baro_t *baro)
{
    static float Temperature,Pressure;        //温度和压强都要读取,因为温度也要补偿气压；
    if (!alt.baro_healthy[0]) {
        return;
    }
    Temperature = spl0601_get_temperature();
    Pressure = spl0601_get_pressure();
    if(!baroIsCalibrationComplete())
    {
        performBaroCalibrationCycle(Pressure);
        baro->altitude = 0.0f;
    }
    else
    {
        //计算去除地面高度后相对高度
        baro->altitude = pressureToAltitude(Pressure) - baroGroundAltitude;
        _baro_altitude = baro->altitude;
    }

    baro->pressure = Pressure;
    baro->temperature = Temperature;

}

bool baroIsCalibrationComplete(void)
{
  return baroCalibrationFinished;
}

void baroStartCalibration(void)
{
    baroCalibrationTimeout = rt_tick_get();
    baroCalibrationFinished = false;
}

//气压值转换为海拔  单位 厘米
 float pressureToAltitude(const float pressure) //输入以帕为单位的数据
{
    float  alt_3 = ( 101000 - pressure ) / 1000.0f;
    return (0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101000 - pressure) * 100.0f);
}

//气压计1个标准大气压校准
 void performBaroCalibrationCycle(float baroPressureSamp)
{
    //慢慢收敛校准
    const float baroGroundPressureError = baroPressureSamp - baroGroundPressure;
    baroGroundPressure += baroGroundPressureError * 0.15f;

    if(absolute(baroGroundPressureError) < (baroGroundPressure * 0.00005f))  // 0.005% calibration error (should give c. 10cm calibration error)
    {
        if( (rt_tick_get() - baroCalibrationTimeout) > 500)
        {
            baroGroundAltitude = pressureToAltitude(baroGroundPressure);
            baroCalibrationFinished = true;
        }
    }
    else
    {
        baroCalibrationTimeout = rt_tick_get();
    }
}


#define PRESSURE_DELTA_GLITCH_THRESHOLD 1000
#define PRESSURE_SAMPLES_MEDIAN 3
 int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;

    int nextSampleIndex = currentFilterSampleIndex + 1;
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN)
    {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }
    int previousSampleIndex = currentFilterSampleIndex - 1;
    if (previousSampleIndex < 0)
    {
        previousSampleIndex = PRESSURE_SAMPLES_MEDIAN - 1;
    }
    const int previousPressureReading = barometerFilterSamples[previousSampleIndex];

    if (medianFilterReady)
    {
        if (absolute(previousPressureReading - newPressureReading) < PRESSURE_DELTA_GLITCH_THRESHOLD)
        {
            barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
            currentFilterSampleIndex = nextSampleIndex;
            return quickMedianFilter3(barometerFilterSamples);
        }
        else
        {
            // glitch threshold exceeded, so just return previous reading and don't add the glitched reading to the filter array
            return barometerFilterSamples[previousSampleIndex];
        }
    }
    else
    {
        barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
        currentFilterSampleIndex = nextSampleIndex;
        return newPressureReading;
    }
}
