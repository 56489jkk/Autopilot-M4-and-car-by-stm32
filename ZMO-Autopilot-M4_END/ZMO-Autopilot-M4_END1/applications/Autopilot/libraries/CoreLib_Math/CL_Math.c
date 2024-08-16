/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#include "CL_Math.h"
#include "string.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>
#include <rtthread.h>
#include <rtdbg.h>
#include <rtdef.h>



Data_change_watch task_watch;
Data_change_watch position_going_watch;//_——_
Data_change_watch height_going_watch;
/**********************************************************************
* 名    称：data_watchdog
* 功    能：监测相邻时间域上同一变量的改变(主要用来监测标志位)
* 入口参数：
* 出口参数：
* 说    明：prev_data初始默认为零，使用函数时需要注意
**********************************************************************/
int8_t data_watchdog(Data_change_watch *watch, bool input)
{
    if (input)
    {
        (*watch).now_data = 1;
    }
    else
    {
        (*watch).now_data = 0;
    }
    if( ((*watch).now_data) !=  ((*watch).prev_data))
    {
        if ((*watch).now_data ==1 )
        {
            (*watch).capture_flag = 1;
        }
        else
        {
            (*watch).capture_flag = -1;
        }
    }
    else
    {
        (*watch).capture_flag = 0;
    }
    (*watch).prev_data = (*watch).now_data;
    return (*watch).capture_flag;
}
/**********************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：求开平方的倒数
**********************************************************************/
float sqrt_reciprocal(float number)//开平方的倒数
{
    long i;
    float x, y;
    x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f3759df - ( i >> 1 );

    y = * ( float * ) &i;
    y = y * ( 1.5f - ( x * y * y ) );
    y = y * ( 1.5f - ( x * y * y ) );

    return y;
}

//快速平方根算法
float my_sqrt(float number)
{
    return number * sqrt_reciprocal(number);
}

/**********************************************************************
* 名    称：absolute
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：求x的绝对值
**********************************************************************/
float absolute(float x)
{
    x = x >0 ? x : -x;
    return x;
}


/**********************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：求两个数的最大值/最小值
**********************************************************************/
float max(float a, float b)
{
    a = a > b ? a : b;
    return a;
}
float min(float a, float b)
{
    a = a > b ? b : a;
    return a;
}


/**********************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：失真限制，大的小的都掐掉
**********************************************************************/
float limit(float data, float min, float max)
{
    data = data > max ? max : data;
    data = data < min ? min : data;
    return data;
}


/**********************************************************************
* 名    称：constrainf
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：不失真限制,但对输入数据有要求，最大最小值为500，-500，主要用于遥控器部分
**********************************************************************/
float constrainf(float data, float low, float high)
{
    if (data < 0)
    {
        return data*low/(-500.0);
    }
    else if (data > 0)
    {
        return data*high/500.0;
    }
    else
    {
        return data;
    }
}


/**********************************************************************
* 名    称：
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：计算一个数的平方
**********************************************************************/
float square(float x)
{
    x = (float)(x * x);
    return x;
}



float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = max(absX, absY);
    if (res) res = min(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (PI / 2.0f) - res;
    if (x < 0) res = PI - res;
    if (y < 0) res = -res;
    return res;
}


float acos_approx(float x)
{
    float xa = fabsf(x);
    float b  = 1.0f - xa;
    float result = sqrtf(b) *  (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return PI - result;
    else
        return result;
}


/***********************传感器校准*************************************/
void devClear(stdev_t *dev)
{
  dev->m_n = 0;
}

void devPush(stdev_t *dev, float x)
{
  dev->m_n++;
  if(dev->m_n == 1)
    {
        dev->m_oldM = dev->m_newM = x;
    dev->m_oldS = 0.0f;
  }
    else
    {
     dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
     dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
     dev->m_oldM = dev->m_newM;
     dev->m_oldS = dev->m_newS;
   }
}

float devVariance(stdev_t *dev)
{
  return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float devStandardDeviation(stdev_t *dev)
{
    return sqrtf(devVariance(dev));
}


/**********************************************************************
* 名    称：float my_deadzone(float x,float ref,float zoom)
* 功    能：添加死区
* 入口参数：x:原始数据 ref:mid-value zoom:dead zone
* 出口参数：data with dead zone added
* 说    明：经过该函数处理，数据输出为-(500-deadband)~0~（500-deadband）
**********************************************************************/
float data_to_deadzone(float x,float ref,float zoom)
{
    float t;
    if(x>ref)
    {
        t = x - zoom;   //x不管在ref~ref+zoom之间怎么变,输出t始终为ref
        if(t<ref)
        {
            t = ref;
        }
    }
    else
    {
        t = x + zoom;
        if(t>ref)
        {
            t = ref;
        }
    }
  return (t);
}

float applyDeadband(float value, float deadband)
{
    if (absolute(value) <= deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

/**********************************************************************
* 名    称：LPF1
* 功    能：
* 入口参数：dt  本函数的调用周期单位ms   *LPF_1   结构体类型LPF_one变量的地址
* 出口参数：
* 说    明：阈值最好是调用周期的整数倍
**********************************************************************/
LPF_one LPF_1[ LPF_all ];
float LPF1(float dt, LPF_one *LPF_1)
{
    /*跟随速度先以20ms 99%跟随为准宏常量跟随时间固定可调*/
    static uint8_t time_threshold;
    time_threshold += dt;
    if(time_threshold >= LPF1_run_T)
    {
        (*LPF_1).output = (*LPF_1).radio * (*LPF_1).input + (1 - (*LPF_1).radio) * (*LPF_1).prev_out;
        (*LPF_1).prev_out = (*LPF_1).output;
        time_threshold = 0;
    }
    return (*LPF_1).output;
}

void LPF1_Reset(LPF_one *LPF_1, int16_t input, float radio)
{
    (*LPF_1).input = input;
    (*LPF_1).output = 0;
    (*LPF_1).prev_out = 0;
    (*LPF_1).radio = radio;
}

Advance_speed advance_height;
Advance_speed advance_position_x;
Advance_speed advance_position_y;
void advance_speed_contral(float dt, Advance_speed *advance)
{
    if (absolute((*advance).speed_buff) < absolute((*advance).speed) && absolute((*advance).different) > (absolute((*advance).original_different)/2.0))        //预加速,只在全程的前半区间
    {
        (*advance).speed_buff += (*advance).speed*dt/1000.0;
    }
    if (absolute((*advance).different) < absolute((*advance).speed)/1.5 && absolute((*advance).speed_buff) > 0 && absolute((*advance).different) <= (absolute((*advance).original_different)/2.0))      //预减速,只在全程的后半区间
    {
        (*advance).speed_buff -= (*advance).speed*dt/1000.0;
    }
}







int32_t quickMedianFilter3(int32_t * v)
{
    int32_t p[3];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int32_t, p[0], p[1]); QMF_SORT(int32_t, p[1], p[2]); QMF_SORT(int32_t, p[0], p[1]) ;
    return p[1];
}

int16_t quickMedianFilter3_16(int16_t * v)
{
    int16_t p[3];
    memcpy(p, v, sizeof(p));

    QMF_SORT(int16_t, p[0], p[1]); QMF_SORT(int16_t, p[1], p[2]); QMF_SORT(int16_t, p[0], p[1]) ;
    return p[1];
}



#define sinPolyCoef3 -1.666665710e-1f                                          // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f                                          // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f                                          // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f                                          // Double:  2.600054767890361277123254766503271638682e-6
/**********************************************************************
* 名    称：sin_approx
* 功    能：
* 入口参数：
* 出口参数：
* 说    明：x单位是弧度
**********************************************************************/
float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  PI) x -= (2.0f * PI);                                 // always wrap input angle to -PI..PI
    while (x < -PI) x += (2.0f * PI);
    if (x >  (0.5f * PI)) x =  (0.5f * PI) - (x - (0.5f * PI));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * PI)) x = -(0.5f * PI) - ((0.5f * PI) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * PI));
}




/************************************end***********************************/

/******************** +++++++++++++++++++++++++++ ********************************
 * 描述    ：快速计算
**********************************************************************************/

#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
//#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256

const float fast_atan_table[257] =
{
    0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
    1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
    3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
    4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
    6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
    7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
    9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
    1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
    1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
    1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
    1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
    1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
    1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
    2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
    2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
    2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
    2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
    2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
    2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
    2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
    3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
    3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
    3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
    3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
    3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
    3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
    3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
    4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
    4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
    4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
    4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
    4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
    4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
    4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
    4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
    5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
    5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
    5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
    5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
    5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
    5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
    5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
    5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
    5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
    6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
    6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
    6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
    6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
    6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
    6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
    6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
    6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
    6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
    6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
    7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
    7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
    7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
    7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
    7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
    7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
    7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
    7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
    7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
    7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
    7.853983e-01
};

float my_abs(float f)
{
    if (f >= 0.0f)
    {
        return f;
    }

    return -f;
}

float fast_atan2(float y, float x)
{
    REAL x_abs, y_abs, z;
    REAL alpha, angle, base_angle;
    int index;

    /* don't divide by zero! */
    if ((y == 0.0f) || (x == 0.0f))//if ((y == 0.0f) && (x == 0.0f))
        angle = 0.0f;
    else
    {
        /* normalize to +/- 45 degree range */
        y_abs = my_abs(y);
        x_abs = my_abs(x);
        //z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
        if (y_abs < x_abs)
            z = y_abs / x_abs;
        else
            z = x_abs / y_abs;
        /* when ratio approaches the table resolution, the angle is */
        /*      best approximated with the argument itself...       */
        if (z < TAN_MAP_RES)
            base_angle = z;
        else
        {
            /* find index and interpolation value */
            alpha = z * (REAL) TAN_MAP_SIZE - .5f;
            index = (int) alpha;
            alpha -= (REAL) index;
            /* determine base angle based on quadrant and */
            /* add or subtract table value from base angle based on quadrant */
            base_angle = fast_atan_table[index];
            base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
        }

        if (x_abs > y_abs)
        {        /* -45 -> 45 or 135 -> 225 */
            if (x >= 0.0f)
            {           /* -45 -> 45 */
                if (y >= 0.0f)
                    angle = base_angle;   /* 0 -> 45, angle OK */
                else
                    angle = -base_angle;  /* -45 -> 0, angle = -angle */
            }
            else
            {                  /* 135 -> 180 or 180 -> -135 */
                angle = 3.14159265358979323846;

                if (y >= 0.0f)
                    angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
                else
                    angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
            }
        }
        else
        {                    /* 45 -> 135 or -135 -> -45 */
            if (y >= 0.0f)
            {           /* 45 -> 135 */
                angle = 1.57079632679489661923;

                if (x >= 0.0f)
                    angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
                else
                    angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
            }
            else
            {                  /* -135 -> -45 */
                angle = -1.57079632679489661923;

                if (x >= 0.0f)
                    angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
                else
                    angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
            }
        }
    }


#ifdef ZERO_TO_TWOPI
    if (angle < 0)
        return (angle + TWOPI);
    else
        return (angle);
#else
    return (angle);
#endif
}

/*
  calculate a low pass filter alpha value
 */
float calc_lowpass_alpha(float dt, float cutoff_freq)
{
    if (dt <= 0.0f || cutoff_freq <= 0.0f) {
        return 1.0;
    }
    float rc = 1.0f/(2*PI*cutoff_freq);
    return limit(dt/(dt+rc), 0.0f, 1.0f);
}


float pythagorous2(float a, float b)
{
  return sqrtf(sq(a)+sq(b));
}


#define M_2PI         (M_PI * 2)
float wrap_PI(const float radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

float wrap_2PI(const float radian)
{
    float res = fmodf(radian, M_2PI);
    if (res < 0) {
        res += M_2PI;
    }
    return res;
}









