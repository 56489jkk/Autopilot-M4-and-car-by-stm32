/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#include "loco_config.h"
#include "CL_Vector.h"
#include "CL_ADRC.h"
#include "CL_PID.h"
void pidInit(pid_struct* pid, float kp, float ki, float kd, int16_t eLimit,int16_t iLimit, int16_t outputLimit, float dt, bool enableDFilter, float cutoffFreq)
{
    pid->target = 0;
    pid->measurement = 0;
    pid->error  = 0;
    pid->error_last = 0;
    pid->integrate = 0;
    pid->derivative = 0;
    pid->output =  0;
    pid->eLimit = eLimit;
    pid->iLimit = iLimit;
    pid->output_limit = outputLimit;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->enableDFilter = enableDFilter;
    if(pid->enableDFilter)
    {
        pid->alpha = calc_lowpass_alpha(dt,cutoffFreq);
    }
    TD_Init(&pid->TD_filter,dt,10000, 5); //跟踪微分器
}
pid_struct pid[Pid_Num];

float pid_rate_updata(pid_struct*pid,float target,float measurement)
{
    static float derivative=0;
    /*******偏差计算*********************/
    pid->target   = target;
    pid->measurement = measurement;
    pid->error = pid->target - pid->measurement;   //期望减去反馈得到偏差
    if(pid->eLimit!=0)  //偏差限幅  当eLimit不为零时，进行偏差限制
    {
        pid->error = limit(pid->error,-pid->eLimit,pid->eLimit);
    }
    /*********************积分计算*********************/
    pid->integrate += pid->dt*pid->error;

    /*****************积分限幅*********************/
    if(pid->iLimit!=0) //积分限幅 当iLimit不为零时，进行积分限制
    {
        pid->integrate = limit(pid->integrate,-pid->iLimit,pid->iLimit);
    }

    //derivative = (pid->measurement_last - pid->measurement)/pid->dt; //微分先行控制器
    derivative = (pid->error - pid->error_last)/pid->dt;
    if(pid->enableDFilter)
    {
        pid->derivative += pid->alpha * (derivative - pid->derivative); //微分项一阶低通
    }
    else pid->derivative =  derivative;
    /****************总输出计算*********************/
    pid->output = pid->Kp*(pid->error)       //比例快速响应
                 +pid->Ki*(pid->integrate)   //积分,消除静态误差
                 +pid->Kd*(pid->derivative);      //微分,抑制作用
    /*******总输出限幅*********************/
    pid->output = limit(pid->output,-pid->output_limit,pid->output_limit);

    pid->measurement_last = pid->measurement;     //保存上次测量
    pid->error_last = pid->error;                 //保存上次偏差

    /*******返回总输出*********************/
    return (pid->output);
}

float pid_pos_updata(pid_struct*pid,float target,float measurement)
{
    static float derivative=0;
    /*******偏差计算*********************/
    pid->target   = target;
    pid->measurement = measurement;
    pid->error = pid->target - pid->measurement;   //期望减去反馈得到偏差
    if(pid->eLimit!=0)  //偏差限幅  当eLimit不为零时，进行偏差限制
    {
        pid->error = limit(pid->error,-pid->eLimit,pid->eLimit);
    }
    /*********************积分计算*********************/
    pid->integrate += pid->dt*pid->error;

    /*****************积分限幅*********************/
    if(pid->iLimit!=0) //积分限幅 当iLimit不为零时，进行积分限制
    {
        pid->integrate = limit(pid->integrate,-pid->iLimit,pid->iLimit);
    }

    //derivative = (pid->measurement_last - pid->measurement)/pid->dt; //微分先行控制器
    derivative = (pid->error - pid->error_last)/pid->dt;
    if(pid->enableDFilter)
    {
        pid->derivative += pid->alpha * (derivative - pid->derivative); //微分项一阶低通
    }
    else pid->derivative =  derivative;
    /****************总输出计算*********************/
    pid->output = pid->Kp*(pid->error)       //比例快速响应
                 +pid->Ki*(pid->integrate)   //积分,消除静态误差
                 +pid->Kd*(pid->derivative);      //微分,抑制作用
    /*******总输出限幅*********************/
    pid->output = limit(pid->output,-pid->output_limit,pid->output_limit);

    pid->measurement_last = pid->measurement;     //保存上次测量
    pid->error_last = pid->error;                 //保存上次偏差

    /*******返回总输出*********************/
    return (pid->output);
}

//角度控制pid
float pid_angle_updata(pid_struct*pid,float target,float measurement,float measurement_dis)
{
    //*******偏差计算*********************/
    TD_Updata(&pid->TD_filter,target);    //跟踪摇杆速度量
    pid->target   = target;
    pid->measurement = measurement;

    pid->error = pid->target - pid->measurement; //期望减去反馈得到偏差
    if(pid->eLimit!=0)//偏差限幅  当eLimit不为零时，进行偏差限制
    {
        pid->error=limit(pid->error,-pid->eLimit,pid->eLimit);
    }

    /*********************积分计算*********************/
    pid->integrate += pid->dt*pid->error;
    /*****************积分限幅*********************/
    if(pid->iLimit!=0) //积分限幅 当iLimit不为零时，进行积分限制
    {
        pid->integrate=limit(pid->integrate,-pid->iLimit,pid->iLimit);
    }

    pid->derivative = pid->TD_filter.x2 - measurement_dis; //摇杆的微分量与角速度做差

    /****************总输出计算*********************/
    pid->output = pid->Kp*(pid->error)        //比例
                 +pid->Ki*(pid->integrate)    //积分
                 +pid->Kd*(pid->derivative);  //微分

    /*******总输出限幅*********************/
    pid->output=limit(pid->output,-pid->output_limit,pid->output_limit);
    pid->measurement_last = pid->measurement;
    pid->error_last = pid->error;             //保存上次偏差
    /*******返回总输出*********************/
    return (pid->output);
}

float pid_yaw_updata(pid_struct*pid,float measurement)
{
    //*******偏差计算*********************/
    pid->measurement = measurement;
    pid->error = pid->target - pid->measurement; //期望减去反馈得到偏差

    //偏航角误差限制，转最小的角度
    if(pid->error>=+180)  pid->error -= 360;
    if(pid->error<=-180)  pid->error += 360;

    /*********************积分计算*********************/
    pid->integrate += pid->dt*pid->error;
    /*****************积分限幅*********************/
    if(pid->iLimit!=0) //积分限幅 当iLimit不为零时，进行积分限制
    {
        pid->integrate = limit(pid->integrate,-pid->iLimit,pid->iLimit);
    }
    pid->derivative = (pid->measurement_last-pid->measurement)/pid->dt; //微分先行控制器

    if(pid->enableDFilter)
    {
    }
    /****************总输出计算*********************/
    pid->output = pid->Kp*(pid->error)//比例
                 +pid->Ki*(pid->integrate)//积分
                 +pid->Kd*(pid->derivative);//微分
    /*******总输出限幅*********************/
    pid->output = limit(pid->output,-pid->output_limit,pid->output_limit);
    pid->measurement_last = pid->measurement;
    pid->error_last = pid->error;//保存上次偏差
    /*******返回总输出*********************/
    return (pid->output);
}

void pid_reset(pid_struct* pid)
{
    pid->target   = 0;
    pid->error      = 0;
    pid->error_last = 0;
    pid->integrate= 0;
    pid->derivative  = 0;
    pid->measurement_last = 0;
    pid->output   = 0;
}

void pid_reset_integral(pid_struct* pid)
{
    pid->integrate  = 0;
}

void pid_set_integral(pid_struct* pid, int integrate)
{
    pid->integrate  = integrate;
}



