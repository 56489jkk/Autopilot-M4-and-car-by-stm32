/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-30     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_ADRC_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_ADRC_H_



typedef struct
{
    //fst跟踪微分器
    float input; //跟踪微分器输入量
    float r;     //（称为速度因子）r越大跟踪速度越快，过大则会使微分项噪声过大
    float h;     //TD系统积分时间
    uint16_t N0; //N0为h的若干整数倍（h0=N0*h）（h0称为滤波因子）
    float fh;    //最速微分器加速度跟踪量
    float x1;    //跟踪微分器状态量
    float x2;    //跟踪微分器状态量微分项
}TD;

typedef struct
{
    float beta_0;//状态误差系数   相当于pid的kp
    float beta_1;//状态的微分误差系数       相当于pid的kd//
    float b0;//扰动系数
    float u; //最终输出
}ADRC;

typedef struct
{
    //状态观测器
    float y;//状态观测器的输入量
    float e;//系统状态误差
    float fe;
    float fe1;
    int16_t beta_01;
    int16_t beta_02;
    int16_t beta_03;
    int16_t   N;
    float h;//ESO系统积分时间
    float z1;
    float z2;//更新输入量的微分
    float z3;//根据控制对象输入与输出，提取的扰动信息
}ESO_Angular_Rate;

void Linear_Conbine_Init(ADRC*L, float beta_0 , float beta_1, float b0);
void TD_Init(TD*filter, float dt, float r , int N0);
void ESO_Init(ESO_Angular_Rate*eso,int32_t beta_01,int32_t beta_02,int32_t beta_03,int N,float dt);
float TD_Updata(TD*filter,float TD_input);//输出状态量的微分量
float ESO_Updata(ESO_Angular_Rate*eso,float ESO_input,float b0 ,float u);

typedef struct
{
    unsigned char tracking_mode;

    float x1;
    float x2;
    float x3;
    float x4;

    float P1;
    float P2;
    float P3;
    float P4;

    float r2p , r2n , r3p , r3n , r4p , r4n;
}TD4;

extern TD4  pos_fil[2];

static inline void TD4_reset( TD4* filter )
{
    filter->x1 = filter->x2 = filter->x3 = filter->x4 = 0;
}

static inline void TD4_setP( TD4* filter , float P )
{
    filter->P1 = filter->P2 = filter->P3 = filter->P4 = P;
}

static inline void TD4_init( TD4* filter , float P1 , float P2 , float P3 , float P4 )
{
    filter->P1 = P1;
    filter->P2 = P2;
    filter->P3 = P3;
    filter->P4 = P4;
    filter->r2p = filter->r2n = filter->r3p = filter->r3n = filter->r4p = filter->r4n = 1e12;
    TD4_reset( filter );
}

static inline float TD4_track4( TD4* filter , const float expect , const float h )
{
    filter->tracking_mode = 4;

    float e1 = expect - filter->x1;
    float e1_1 = -filter->x2;
    float e1_2 = -filter->x3;
    float e1_3 = -filter->x4;
    float T2 = filter->P1 * e1;
    float P1 = 0;
    if( T2 > filter->r2p )
        T2 = filter->r2p;
    else if( T2 < -filter->r2n )
        T2 = -filter->r2n;
    else
        P1 = filter->P1;
    float T2_1 = P1 * e1_1;
    float T2_2 = P1 * e1_2;
    float T2_3 = P1 * e1_3;

    float e2 = T2 - filter->x2;
    float e2_1 = T2_1-filter->x3;
    float e2_2 = T2_2-filter->x4;
    float T3 = filter->P2 * e2;
    float P2 = 0;
    if( T3 > filter->r3p )
        T3 = filter->r3p;
    else if( T3 < -filter->r3n )
        T3 = -filter->r3n;
    else
        P2 = filter->P2;
    T3 += T2_1;
    float T3_1 = P2 * e2_1 + T2_2;
    float T3_2 = P2 * e2_2 + T2_3;

    float e3 = T3 - filter->x3;
    float e3_1 = T3_1-filter->x4;
    float T4 = filter->P3 * e3;
    float P3 = 0;
    if( T4 > filter->r4p )
        T4 = filter->r4p;
    else if( T4 < -filter->r4n )
        T4 = -filter->r4n;
    else
        P3 = filter->P3;
    T4 += T3_1;
    float T4_1 = P3 * e3_1 + T3_2;

    float e4 = T4 - filter->x4;
    float T5 = filter->P4 * e4 + T4_1;

    filter->x1 += h*filter->x2;
    filter->x2 += h*filter->x3;
    filter->x3 += h*filter->x4;
    filter->x4 += h*T5;

    return filter->x1;

}

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_PID_CL_ADRC_H_ */
