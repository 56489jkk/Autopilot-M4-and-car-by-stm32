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
#include "CL_NavigationKF.h"
#include "CL_Vector.h"
#include "GCS_Communication.h"
#include "DL_LinkTrack_P_BS2.h"
Kalman_Filter_t nav;
Kalman_Filter_t INS;
Kalman_Filter_t Flow_INS,T265_INS,Radar_INS,UWB_INS;

Vector3f  _body_frame_acce,_nav_earth_acce,earth_frame_acce_g;
Vector3f get_navigation_body_frame_acce()  //机头方向为 +X
{
    return _body_frame_acce ;
}
Vector3f get_navigation_earth_frame_acce() //NED坐标系 （北东地坐标系）
{
    return _nav_earth_acce;
}

uint8_t _pos_vel_type=0;

uint8_t get_pos_vel_type()
{
    return _pos_vel_type;
}

void navigation_accel_update(Vector3f accBF)
{
    //旋转机体坐标系加速度到世界坐标系
    imu_transform_vector_body_to_earth(&accBF,&earth_frame_acce_g);

    earth_frame_acce_g.z += 980;     //去除重力
    for (int i = 0; i < 3; i++)
    {
        nav.earth_acce.axis[i] = earth_frame_acce_g.axis[i];
        _nav_earth_acce = nav.earth_acce;                    //地理坐标系下的导航加速度
    }

    //计算机体运动加速度
    //_body_frame_acce.x = 俯仰方向上的运动加速度  向前运动为正方向
    //_body_frame_acce.y = 横滚方向上的运动加速度  向右运动为正方向
    imu_transform_vector_earth_to_body(&_body_frame_acce,&nav.earth_acce);
}

bool get_vision_health()
{
    if(rt_tick_get() - sysid_myggcs_last_seen_time_ms() > 5*100){
        return  false;
    }
    else  return  true;
}

float get_current_ranging_altitude()  //cm
{
    float altitude_data = 0;
    altitude_data = get_opt_distance()*rMat[2][2];
    //altitude_data = get_baro_sensorsAltitude();     //高度数据的来源（气压计，超声波，激光等）

    return  altitude_data;
}

Vector3f _relpos_cm,_velocity_cms;
void position_estimation_update(float dt)
{
    static uint8_t isInitialized = 0;

    static float Q[4]={0.03,0.00,0.1,0.03};      //过程噪声方差//{0.001,0.01,0.01,0.005};//过程噪声方差
    static float R[2]={150,0};                   //30  100   150  300 观测噪声方差
    static float Pre_cov[4]={0.18,0.1,0.1,0.18}; //上一次协方差

    static float Q_Flow[4]={0.1,0,0.6,0.1};//{0.075,3.0};{0.015,3.0}//过程噪声方差
    static float R_Flow[2]={60,0};       //30  100   150  300 观测噪声方差
    static float Pre_cov_Flow[4]={0.018 , 0.005, 0.005, 0.5};//0.0001 ,    0.00001,  0.00001    , 0.003,

    static float Q_T265[4]={0.08,0,0.001,0.08};//{0.075,3.0};{0.015,3.0}//过程噪声方差
    static float R_T265[2]={160,0};       //30  100   150  300 观测噪声方差
    static float Pre_cov_T265[4]={0.018 , 0.005, 0.005, 0.5};//0.0001 ,    0.00001,  0.00001    , 0.003,

    static float Q_radar[4]={0.3,0.00,0.1,0.3};   //过程噪声方差//{0.001,0.01,0.01,0.005};//过程噪声方差
    static float R_radar[2]={20,0};                 //30  100   150  300 观测噪声方差
    static float Pre_cov_radar[4]={0.18,0.1,0.1,0.18};     //上一次协方差

    static float Q_uwb[4]={0.3,0.00,0.1,0.3};     //过程噪声方差//{0.001,0.01,0.01,0.005};//过程噪声方差
    static float R_uwb[2]={20,0};                 //30  100   150  300 观测噪声方差
    static float Pre_cov_uwb[4]={0.18,0.1,0.1,0.18};     //上一次协方差
    mavlink_vision_position_estimate_t vision_estimate = get_vision_position_estimate();
    //初始化预估器
    if(!isInitialized)
    {
        kalman_filter_init(&INS,R,Q,Pre_cov);                      //高度卡尔曼参数初始化
        kalman_filter_init(&Flow_INS,R_Flow,Q_Flow,Pre_cov_Flow);  //光流卡尔曼参数初始化
        kalman_filter_init(&T265_INS,R_T265,Q_T265,Pre_cov_T265);  //卡尔曼参数初始化

        kalman_filter_init(&Radar_INS,R_radar,Q_radar,Pre_cov_radar); //激光雷达卡尔曼参数初始化
        Radar_INS.Position.x = vision_estimate.x;
        Radar_INS.Position.y = vision_estimate.y;
        Radar_INS.Position.z = vision_estimate.z;


        kalman_filter_init(&UWB_INS,R_uwb,Q_uwb,Pre_cov_uwb); //uwb卡尔曼参数初始化
        UWB_INS.Position.x = UWB.raw_px*100;
        UWB_INS.Position.y = UWB.raw_py*100;

        rt_thread_mdelay(3000);                         //调度
        INS.Position.z = get_current_ranging_altitude();
        //INS.Position.z = get_baro_sensorsAltitude();  //高度数据的来源（气压计，超声波，激光等）
        isInitialized = 1;
        biquadFilterInitLPF(&t265_pos_LPF[0],1/dt,15);
        biquadFilterInitLPF(&t265_pos_LPF[1],1/dt,15);
    }

    float altitude_data = get_current_ranging_altitude();   //单位 cm;   //高度数据的来源（气压计，超声波，激光等）
    //float altitude_data = get_baro_sensorsAltitude();     //高度数据的来源（气压计，超声波，激光等）

    Vector3f earth_acce = get_navigation_earth_frame_acce();
    Kalman_Filter_updata_1(&INS,-altitude_data,earth_acce.z,dt,0,'Z');

    Vector3f body_acce = get_navigation_body_frame_acce();
    Vector2f flow_pos = opticalflow_rotate_complementary_filter(dt);
    Kalman_Filter_updata_1(&Flow_INS,flow_pos.x,body_acce.x,dt,0,'X');
    Kalman_Filter_updata_1(&Flow_INS,flow_pos.y,body_acce.y,dt,0,'Y');

    Vector2f t265_vel = t265_rotate_complementary_filter(dt);
    Kalman_Filter_updata_2(&T265_INS,t265_vel.x,body_acce.x,dt,0,'X');
    Kalman_Filter_updata_2(&T265_INS,t265_vel.y,body_acce.y,dt,0,'Y');

//    Vector2f t265_pos;
//    t265_pos.x = biquadFilterApply(&t265_pos_LPF[0],t265_estimate.pos_x); //
//    t265_pos.y = biquadFilterApply(&t265_pos_LPF[1],t265_estimate.pos_y); //


    //Vector3f body_acce = get_navigation_body_frame_acce();//机体的加速度
    float r_x = cosf(vision_estimate.yaw)*body_acce.x + sinf(vision_estimate.yaw)*body_acce.y;  //雷达的世界坐标系下  向前为正
    float r_y = sinf(vision_estimate.yaw)*body_acce.x - cosf(vision_estimate.yaw)*body_acce.y;  //雷达的世界坐标系下  向左为正

    Kalman_Filter_updata_1(&Radar_INS,vision_estimate.x*100,r_x,dt,0,'X');  //cm
    Kalman_Filter_updata_1(&Radar_INS,vision_estimate.y*100,r_y,dt,0,'Y');  //cm
    Kalman_Filter_updata_1(&Radar_INS,vision_estimate.z*100,earth_acce.z,dt,0,'Z');  //cm


//    Kalman_Filter_updata_1(&UWB_INS,UWB.raw_px*100,earth_acce.y,dt,0,'X');  //cm     uwb坐标系  raw_px向右为正
//    Kalman_Filter_updata_1(&UWB_INS,UWB.raw_py*100,earth_acce.x,dt,0,'Y');  //cm     uwb坐标系  raw_py向前为正
//    Kalman_Filter_updata_1(&T265_INS,t265_position_estimate.y*100,body_acce.x,dt,0,'X');
//    Kalman_Filter_updata_1(&T265_INS,t265_position_estimate.x*100,body_acce.y,dt,0,'Y');
//    t265_pos.x = biquadFilterApply(&t265_pos_LPF[0],t265_position_estimate.y*100); //
//    t265_pos.y = biquadFilterApply(&t265_pos_LPF[1],t265_position_estimate.x*100); //

    _relpos_cm.z = -INS.Position.z; // convert from m in NED to cm in NEU
    _velocity_cms.z = -INS.Speed.z; // convert from m in NED to cm in NEU

    //static uint8_t t265_ok = 0;
//    if(get_t265_health()&&1) {
//        _relpos_cm.x = t265_pos.x;
//        _relpos_cm.y = t265_pos.y;
//        _velocity_cms.x = T265_INS.Speed.x;
//        _velocity_cms.y = T265_INS.Speed.y;
//        _pos_vel_type = 1;
//    }
    if(get_vision_health()||_pos_vel_type==1) {
        _relpos_cm.x = Radar_INS.Position.x;
        _relpos_cm.y = Radar_INS.Position.y;
        _velocity_cms.x = Radar_INS.Speed.x;
        _velocity_cms.y = Radar_INS.Speed.y;
        _pos_vel_type = 1;
    }
    else
    {
        _relpos_cm.x = Flow_INS.Position.x;
        _relpos_cm.y = Flow_INS.Position.y;
        _velocity_cms.x = Flow_INS.Speed.x;
        _velocity_cms.y = Flow_INS.Speed.y;
        _pos_vel_type = 2;
    }
}


Vector3f get_position_neu_cm(void) //返回当前位置
{
    return _relpos_cm;
}
Vector3f get_velocity_neu_cms(void) //返回当前速度
{
    return _velocity_cms;
}
float get_position_z_up_cm(void) //返回当前高度
{
    return _relpos_cm.z;
}
float get_velocity_z_up_cms(void) //返回当前高度方向的速度
{
    return _velocity_cms.z;
}



/****************************************************************************************
*@brief   卡尔曼滤波参数初始化
*@param[in]
*****************************************************************************************/
void kalman_filter_init(Kalman_Filter_t*Filter,float*R,float*Q,float*Pre_cov)
{
    Filter->R[0] = R[0];
    Filter->R[1] = R[1];
    Filter->Q[0] = Q[0];
    Filter->Q[1] = Q[1];
    Filter->Q[2] = Q[2];
    Filter->Q[3] = Q[3];
    Filter->Pre_cov[0] = Pre_cov[0];
    Filter->Pre_cov[1] = Pre_cov[1];
    Filter->Pre_cov[2] = Pre_cov[2];
    Filter->Pre_cov[3] = Pre_cov[3];
    for (uint8_t i = 0; i < 3; ++i)
    {
        Filter->Position.axis[i]=0;
        Filter->Speed.axis[i]=0;
        Filter->Acce.axis[i]=0;
        Filter->Acce_Bias.axis[i]=0;
    }

}

void  Kalman_Filter_updata_1(Kalman_Filter_t *Pilot, //惯性导航结构体
                             float Observation,      //速度观测量
                             float accel,            //系统原始驱动量,惯导加速度
                             float dt,               //系统积分步长
                             uint8_t *update_flag,   //数据更新标志
                             uint8_t Axis)           //坐标系,必须为（"X","Y","Z",）其中之一
{
    uint8_t i=0;
    double Temp_cov[4]={0},k[2]={0};//增益矩阵;//先验协方差
    if(Axis =='X') i=0;
    if(Axis =='Y') i=1;
    if(Axis =='Z') i=2;

    /*先验状态 Xn=F*X(t-1)+B*U;%状态预测公式
    F为状态转移矩阵  X(t-1)为前一次预测的状态矩阵 B为控制矩阵
       1   dt         Pilot->Pos.z     dt*dt/2
    F=       , X(t-1)=              ,B=           U=Acce
       0   1          Pilot->Speed.z    dt                  */

    Pilot->Acce.axis[i]     = Pilot->Acce_Bias.axis[i] + accel;
    Pilot->Speed.axis[i]    = Pilot->Speed.axis[i] + Pilot->Acce.axis[i]*dt;
    Pilot->Position.axis[i] = Pilot->Position.axis[i] + Pilot->Speed.axis[i]*dt + (Pilot->Acce.axis[i]*dt*dt)/2.0f;

    /*先验协方差 Temp_cov=F * Pre_cov * F'+Q;求先验协方差矩阵的公式
    F为状态转移矩阵 F'为状态转移矩阵的转置矩阵，Pre_cov为前一次协方差矩阵
    Q为过程噪声方差矩阵*/
    Temp_cov[0] = Pilot->Pre_cov[0] + Pilot->Pre_cov[2]*dt + (Pilot->Pre_cov[1] + Pilot->Pre_cov[3]*dt)*dt + Pilot->Q[0];
    Temp_cov[1] = Pilot->Pre_cov[1] + Pilot->Pre_cov[3]*dt + Pilot->Q[1];
    Temp_cov[2] = Pilot->Pre_cov[2] + Pilot->Pre_cov[3]*dt + Pilot->Q[2];
    Temp_cov[3] = Pilot->Pre_cov[3] + Pilot->Q[3];

    /*计算卡尔曼增益  K=Temp_cov*H'*(H*Temp_cov*H'+R)^-1;
    K为卡尔曼增益矩阵，Temp_cov求出的协方差矩阵，H为观测矩阵，H'为观测矩阵的转置矩阵
    (X)^-1为求X矩阵的逆矩阵  R为观测噪声                           1
                                      H=
                                           0      */
    k[0]=Temp_cov[0]/(Temp_cov[0]+Pilot->R[0]);
    k[1]=Temp_cov[2]/(Temp_cov[0]+Pilot->R[0]);

    //计算测量的高度与估计到的高度之差
    //X= X(t-1)+K*(Y-H*X(t-1));
    //Y为状态观测矩阵  这里的矩阵Y只有一个数字  即测到的高度
    //这里Err=(Y-H*X(t-1))
    float Err = Observation - Pilot->Position.axis[i];

    Pilot->Speed.axis[i]    = Pilot->Speed.axis[i] + k[1]*Err;
    Pilot->Position.axis[i] = Pilot->Position.axis[i] + k[0]*Err;
    Pilot->Acce_Bias.axis[i] += 0.000f*Err;
    Pilot->Acce_Bias.axis[i] =  limit(Pilot->Acce_Bias.axis[i],-50,+50);

    //更新状态协方差矩阵
    //Pre_cov=(I-K*H)*Temp_cov;  I为单位矩阵
    Pilot->Pre_cov[0]=(1-k[0])*Temp_cov[0];
    Pilot->Pre_cov[1]=(1-k[0])*Temp_cov[1];
    Pilot->Pre_cov[2]=Temp_cov[2]-k[1]*Temp_cov[0];
    Pilot->Pre_cov[3]=Temp_cov[3]-k[1]*Temp_cov[1];
}

/****************************************************************************************
*@brief     光流卡尔曼滤波
*@brief     输出俯仰和横滚方向的速度和位移       以前和右为正方向
*@param[in] 俯仰和横滚的速度、位移、运动加速度
*****************************************************************************************/
void  Kalman_Filter_updata_2(Kalman_Filter_t *Pilot,//惯性导航结构体
                             float Vel_Observation, //速度观测量
                             float accel,           //系统原始驱动量，惯导加速度
                             float dt,              //系统积分步长
                             uint8_t *update_flag,  //数据更新标志
                             uint8_t Axis)          //坐标系，必须为（"X","Y","Z",）其中之一
{
    uint8_t i=0;
    double Temp_cov[4]={0},k[2]={0};//增益矩阵;//先验协方差
    if(Axis =='X') i=0;
    if(Axis =='Y') i=1;
    if(Axis =='Z') i=2;
    //先验状态
    Pilot->Acce.axis[i]     = Pilot->Acce_Bias.axis[i] + accel;
    Pilot->Speed.axis[i]    = Pilot->Speed.axis[i] + Pilot->Acce.axis[i]*dt;
    Pilot->Position.axis[i] = Pilot->Position.axis[i] + Pilot->Speed.axis[i]*dt + (Pilot->Acce.axis[i]*dt*dt)/2.0f;
    //先验协方差
    Temp_cov[0] = Pilot->Pre_cov[0] + Pilot->Pre_cov[2]*dt + (Pilot->Pre_cov[1] + Pilot->Pre_cov[3]*dt)*dt + Pilot->Q[0];
    Temp_cov[1] = Pilot->Pre_cov[1] + Pilot->Pre_cov[3]*dt + Pilot->Q[1];
    Temp_cov[2] = Pilot->Pre_cov[2] + Pilot->Pre_cov[3]*dt + Pilot->Q[2];
    Temp_cov[3] = Pilot->Pre_cov[3] + Pilot->Q[3];
    //计算卡尔曼增益
    //化简如下
    k[0]=Temp_cov[1]/(Temp_cov[3]+Pilot->R[0]);
    k[1]=Temp_cov[2]/(Temp_cov[3]+Pilot->R[0]);

    //融合数据输出
    float Err = Vel_Observation - Pilot->Speed.axis[i];

    Pilot->Position.axis[i]   += k[0]*Err;
    Pilot->Speed.axis[i] += k[1]*Err;
    Pilot->Acce_Bias.axis[i] += 0.000*Err;
    Pilot->Acce_Bias.axis[i] = limit(Pilot->Acce_Bias.axis[i],-50,50);
    //更新状态协方差矩阵
    Pilot->Pre_cov[0]=Temp_cov[0]-k[0]*Temp_cov[2];
    Pilot->Pre_cov[1]=Temp_cov[1]-k[0]*Temp_cov[3];
    Pilot->Pre_cov[2]=(1-k[1])*Temp_cov[2];
    Pilot->Pre_cov[3]=(1-k[1])*Temp_cov[3];

}


float pos_correction[3]={0,0,0};
float acc_correction[3]={0,0,0};
float vel_correction[3]={0,0,0};
float SpeedDealt[3]={0};

/****气压计三阶互补滤波方案——参考开源飞控APM****/
#define TIME_CONTANST_ZER    2.0f
#define K_ACC_ZER           (0.1f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))//1
#define K_VEL_ZER           (50.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//3
#define K_POS_ZER           (25.0f / TIME_CONTANST_ZER)//3
void Strapdown_INS_Pos(Kalman_Filter_t *Pilot,  //惯性导航结构体
                        float Observation,      //速度观测量
                        float accel,            //系统原始驱动量,惯导加速度
                        float dt,               //系统积分步长
                        uint16_t Pos_Delay_Cnt, //观测传感器延时量
                        uint8_t *update_flag,   //数据更新标志
                        uint8_t Axis)           //坐标系,必须为（"X","Y","Z",）其中之一
{
    uint8_t i=0;
    if(Axis =='X') i=0;
    if(Axis =='Y') i=1;
    if(Axis =='Z') i=2;
    static uint16_t Speed_Sync_Cnt=0,Pos_Sync_Cnt=0;
    uint16_t Cnt=0;
    float Altitude_Dealt[3]={0,0,0};
    //由观测量得到状态误差
    Altitude_Dealt[i] = (Observation - Pilot->Pos_History[i][Pos_Delay_Cnt]); //气压计(超声波)与SINS估计量的差，单位cm
    //三路积分反馈量修正惯导
    acc_correction[i] += Altitude_Dealt[i]* K_ACC_ZER*dt ; //加速度矫正量
    vel_correction[i] += Altitude_Dealt[i]* K_VEL_ZER*dt ; //速度矫正量
    pos_correction[i] += Altitude_Dealt[i]* K_POS_ZER*dt ; //位置矫正量
    //加速度计矫正后更新
    Pilot->last_Acce.axis[i] = Pilot->Acce.axis[i];//上一次加速度量
    Pilot->Acce.axis[i]= accel + acc_correction[i];
    //速度增量矫正后更新,用于更新位置,由于步长h=0.005,相对较长，
    //这里采用二阶龙格库塔法更新微分方程,不建议用更高阶,因为加速度信号非平滑
    SpeedDealt[i] = (Pilot->last_Acce.axis[i] + Pilot->Acce.axis[i])*dt/2.0f;

    Pilot->Ori_Position.axis[i] += (Pilot->Speed.axis[i] + 0.5f*SpeedDealt[i])*dt; //原始位置更新
    Pilot->Position.axis[i] = Pilot->Ori_Position.axis[i] + pos_correction[i]; //位置矫正后更新

    Pilot->Ori_Speed.axis[i] += SpeedDealt[i]*0.5; //原始速度更新
    Pilot->Speed.axis[i] = Pilot->Ori_Speed.axis[i] + vel_correction[i]; //速度矫正后更新

    Pos_Sync_Cnt++;
    if(Pos_Sync_Cnt>=2) //10ms滑动一次
    {
        for(Cnt=Num-1;Cnt>0;Cnt--)
        {
            Pilot->Pos_History[i][Cnt] = Pilot->Pos_History[i][Cnt-1];
        }
        Pos_Sync_Cnt=0;
    }
    Pilot->Pos_History[i][0] = Pilot->Position.axis[i];


    Speed_Sync_Cnt++;
    if(Speed_Sync_Cnt>=2) //10ms滑动一次
    {
        for(Cnt=Num-1;Cnt>0;Cnt--)
        {
            Pilot->Vel_History[i][Cnt] = Pilot->Vel_History[i][Cnt-1];
        }
        Speed_Sync_Cnt=0;
    }
    Pilot->Vel_History[i][0] = Pilot->Speed.axis[i];
}




