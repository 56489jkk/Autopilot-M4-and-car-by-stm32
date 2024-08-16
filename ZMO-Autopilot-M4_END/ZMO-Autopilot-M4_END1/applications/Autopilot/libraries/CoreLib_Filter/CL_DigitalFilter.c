/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#include "loco_config.h"
#include "math.h"
#include "CL_DigitalFilter.h"

/*********************************************************主要为巴特沃斯滤波器***********************************************************/

biquadFilter_t gyroFilterLPF[3],acceFilterLPF[3],pilot_acce_LPF[3],baro_LPF,flow_LPF[3],gyro_LPF[3],t265_LPF[3],t265_gyro_LPF[3],t265_pos_LPF[3],Thr_base,baro_lpf;
#define BIQUAD_BANDWIDTH 1.9f           /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

/****************************************************************************************
*@brief   滤波参数初始化
*@param[in]  采样频率   截止频率
*****************************************************************************************/
void Filter_Parameter_Init(void)
{
    int i=0;
    for(i=0;i<3;i++)
    {
        biquadFilterInitLPF(&flow_LPF[i],200,20);
        biquadFilterInitLPF(&gyro_LPF[i],200,10);
        biquadFilterInitLPF(&Thr_base,100,10);
        biquadFilterInitLPF(&pilot_acce_LPF[i],500,15);

        biquadFilterInitLPF(&gyroFilterLPF[i],500,60);
        biquadFilterInitLPF(&acceFilterLPF[i],500,20);
    }
}

float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff)
{
    const float octaves = log2f((float)centerFreq  / (float)cutoff) * 2;
    return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
}

//二阶陷波器
void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, samplingFreq, filterFreq, Q, FILTER_NOTCH);
}

//二阶低通滤波器
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq)
{
    biquadFilterInit(filter, samplingFreq, filterFreq, BIQUAD_Q, FILTER_LPF);
}

//二阶滤波器
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (samplingFreq / 2))
    {
        // setup variables
        const float sampleRate = samplingFreq;
        const float omega = 2.0f * PI * ((float)filterFreq) / sampleRate;
        const float sn = sin_approx(omega);
        const float cs = cos_approx(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType) {
        case FILTER_LPF:
            b0 = (1 - cs) / 2;
            b1 =  1 - cs;
            b2 = (1 - cs) / 2;
            break;
        case FILTER_NOTCH:
            b0 =  1;
            b1 = -2 * cs;
            b2 =  1;
            break;
        }
        const float a0 =  1 + alpha;
        const float a1 = -2 * cs;
        const float a2 =  1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    }
    else
    {
        // Not possible to filter frequencies above Nyquist frequency - passthrough
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
    }

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}

// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}

#define FILTER_NUM  5
#define FILTER_A    20.0f

/*限幅平均滤波法*/
float pressureFilter(float in)
{
    static uint8_t i=0;
    float out;
    static float filter_buf[FILTER_NUM]={0.0};
    double filter_sum=0.0;
    uint8_t cnt=0;
    float deta;
    if(filter_buf[i] == 0.0f)
    {
        filter_buf[i]=in;
        out=in;
        if(++i>=FILTER_NUM) i=0;
    }
    else
    {
        if(i) deta=in-filter_buf[i-1];
        else deta=in-filter_buf[FILTER_NUM-1];

        if(fabs(deta)<FILTER_A)
        {
            filter_buf[i]=in;
            if(++i>=FILTER_NUM) i=0;
        }
        for(cnt=0;cnt<FILTER_NUM;cnt++)
        {
            filter_sum+=filter_buf[cnt];
        }
        out=filter_sum /FILTER_NUM;
    }
    return out;
}


