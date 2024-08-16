/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_FILTER_CL_DIGITALFILTER_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_FILTER_CL_DIGITALFILTER_H_

#include "loco_config.h"
typedef struct rateLimitFilter_s {
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_FIR,
} filterType_e;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;

void Filter_Parameter_Init(void);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint16_t f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);

void rateLimitFilterInit(rateLimitFilter_t *filter);
float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT);

void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
float firFilterApply(const firFilter_t *filter);

/*限幅平均滤波法*/
float pressureFilter(float in);

extern biquadFilter_t gyroFilterLPF[3],acceFilterLPF[3],pilot_acce_LPF[3],baro_LPF,flow_LPF[3],gyro_LPF[3],t265_LPF[3],t265_gyro_LPF[3],t265_pos_LPF[3],Thr_base;

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_FILTER_CL_DIGITALFILTER_H_ */
