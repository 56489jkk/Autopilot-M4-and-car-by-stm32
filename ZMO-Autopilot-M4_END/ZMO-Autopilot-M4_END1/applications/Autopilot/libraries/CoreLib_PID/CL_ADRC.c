/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-30     CGY       the first version
 */
#include "CL_Math.h"
#include "CL_Vector.h"
#include "CL_ADRC.h"
#include "CL_PID.h"
#include "math.h"
int16_t Sign(double Input)  //符号函数
{
  int16_t output=0;
  if(Input>0) output=1;
  else if(Input<0) output=-1;
  else output=0;
  return output;
}


int16_t Fsg(float x,float d)
{
  int16_t output=0;
  output=(Sign(x+d)-Sign(x-d))/2;
  return output;
}

//原点附近有连线性段的连续幂次函数
float Fal(float e,float alpha,float zeta)
{
  uint16_t s=0;
  float fal_output=0;
  s=(Sign(e+zeta)-Sign(e-zeta))/2;
  fal_output=e*s/(powf(zeta,1-alpha))+powf(absolute(e),alpha)*Sign(e)*(1-s);
  return fal_output;
}

void TD_Init(TD*filter, float dt, float r , int N0)
{
  filter->input = 0;
  filter->h  = dt   ;
  filter->N0 = N0;
  filter->r  = r;
  filter->fh =0;
  filter->x1 =0;
  filter->x2 =0;
}

void ESO_Init(ESO_Angular_Rate*eso,int32_t beta_01,int32_t beta_02,int32_t beta_03,int N,float dt)
{
    eso->y=0;
    eso->e=0;
    eso->fe=0;
    eso->beta_01= beta_01;
    eso->beta_02= beta_02;
    eso->beta_03= beta_03;
    eso->z1 =0;
    eso->z2 =0;
    eso->z3 =0;
    eso->h  =dt;
    eso->N  =N;
}


//跟踪微分器    输入数据，   得到滤波后的数据和数据的微分
float TD_Updata(TD*filter,float TD_input)//输出状态量的微分量
{
  //TD状态跟踪误差项   h0=N0*h
  float x1_delta= filter->x1 - TD_input;//用x1-v(k)替代x1得到离散更新公式
  float d=filter->r*(filter->h*filter->N0);//d=r*h0;
  float d0=(filter->h*filter->N0)*d;//d0=d*h0
  float y=x1_delta + (filter->h*filter->N0)*filter->x2;//y=x1_delta+h0*x2
  float a0=sqrt(d*d + 8*filter->r*absolute(y));//a1=sqrt(d*(d+8*ABS(y))])
  float a=(filter->x2+y/(filter->h*filter->N0))*Fsg(y,d0) + (filter->x2+(a0-d)/2*Sign(y))*(1 - Fsg(y,d0));
  filter->fh=-filter->r*(a/d)*Fsg(a,d) - filter->r*Sign(a)*(1-Fsg(a,d)); //得到最速微分加速度跟踪量
  filter->x1+=filter->h*filter->x2; //跟新最速跟踪状态量x1
  filter->x2+=filter->h*filter->fh; //跟新最速跟踪状态量微分x2
  return filter->x1;
}

float ESO_Updata(ESO_Angular_Rate*eso,float ESO_input,float b0 ,float u)
{
  eso->y=ESO_input;
  eso->e=eso->z1-eso->y;//状态误差
  eso->fe =Fal(eso->e,0.5, eso->N*eso->h);//非线性函数，提取跟踪状态与当前状态误差
  eso->fe1=Fal(eso->e,0.25,eso->N*eso->h);
  /*************扩展状态量更新**********/
  eso->z1+=eso->h*(-eso->beta_01*eso->e + eso->z2);    //更新输入量
  eso->z2+=eso->h*(-eso->beta_02*eso->fe+ eso->z3+b0*u);  //更新输入量的微分
  eso->z3+=eso->h*(-eso->beta_03*eso->fe1);           //更新输入量的总扰动
  return eso->z1;
}

void Linear_Conbine_Init(ADRC*L, float beta_0 , float beta_1, float b0)
{
    L->u  = 0;
    L->beta_0 = beta_0; //相当于pid的kp
    L->beta_1 = beta_1; //相当于pid的kd
    L->b0 = b0;         //相当于pid的ki,但又不相似
}
