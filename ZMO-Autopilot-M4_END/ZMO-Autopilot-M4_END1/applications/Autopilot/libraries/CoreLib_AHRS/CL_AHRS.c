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
#include "CL_Vector.h"
#include "CL_AHRS.h"
#include "math.h"


/* 消息队列控制块 */
static struct rt_messagequeue ahrsDataQueue;
static char msgAhrs_pool[2048];

int ahrs_DataQueue_init(void)
{
    /* 初始化消息队列 */
    rt_mq_init(&ahrsDataQueue, "mqAhrs", &msgAhrs_pool[0],sizeof(attitude_t), sizeof(msgAhrs_pool), RT_IPC_FLAG_FIFO);
    return RT_EOK;
}

INIT_APP_EXPORT(ahrs_DataQueue_init);//自动加入初始化

/*从队列读取欧拉角数据*/
bool ahrsReadEulerAngles(attitude_t *attitude)  //为四元数姿态解算得到的欧拉角  单位：度
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&ahrsDataQueue,attitude, sizeof(attitude_t), 0)== RT_EOK);
}

attitude_t _attitude_angle;
attitude_t get_ahrs_eulerAngles(void)  //为四元数姿态解算得到的欧拉角  单位：度
{
    return _attitude_angle;
}

int get_ahrs_queue_entry()
{
    /* 队列中已有的消息数 */
    return ahrsDataQueue.entry;
}

/*************************H7飞控新坐标下的姿态解算******************************************************/
/*****************************************************************************************************/
#define DCM_KP_ACC          0.400f      //加速度补偿陀螺仪PI参数
#define DCM_KI_ACC          0.005f

#define DCM_KP_MAG          1.000f      //磁力计补偿陀螺仪PI参数
#define DCM_KI_MAG          0.000f

#define IMU_SMALL_ANGLE     15.0f       //满足水平状态的最小角度（单位deg）
#define Spin_Rate_Limit     20          //旋转速率

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
ahrs_t ahrs = {1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
float rMat[3][3]; //四元数的旋转矩阵
//float smallAngleCosZ; //水平最小角余弦值
float imuAttitudeYaw; //范围: -180 180,用于上传到匿名上位机(支持范围-180~180)
ahrs_t Quaternion;
/**********************************************************************
  * @brief  计算四元数的旋转矩阵
  * @retval 无
**********************************************************************/
static void imu_compute_rotation_matrix(void)
{
    float q1q1 = ahrs.q1 * ahrs.q1;
    float q2q2 = ahrs.q2 * ahrs.q2;
    float q3q3 = ahrs.q3 * ahrs.q3;

    float q0q1 = ahrs.q0 * ahrs.q1;
    float q0q2 = ahrs.q0 * ahrs.q2;
    float q0q3 = ahrs.q0 * ahrs.q3;
    float q1q2 = ahrs.q1 * ahrs.q2;
    float q1q3 = ahrs.q1 * ahrs.q3;
    float q2q3 = ahrs.q2 * ahrs.q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/***********************************************************
*@brief ：载体系向导航系转换 （导航坐标系下，可把飞行器看做一个质点）
*************************************************************/
void imu_transform_vector_body_to_earth(Vector3f *bf,Vector3f *ef)
{
  ef->x = rMat[0][0]*bf->x + rMat[0][1]*bf->y + rMat[0][2]*bf->z;  //南北方向的加速度    向北运动为正
  ef->y = rMat[1][0]*bf->x + rMat[1][1]*bf->y + rMat[1][2]*bf->z;  //东西方向的加速度    向东运动为正
  ef->z = rMat[2][0]*bf->x + rMat[2][1]*bf->y + rMat[2][2]*bf->z;  //起降加速度          向地运动为正
}

/***********************************************************
*@brief ：导航系向载体系转换（载体坐标系下，在飞行器上建立坐标系）
*************************************************************/
void imu_transform_vector_earth_to_body(Vector3f *bf,Vector3f *ef)
{
  bf->x = rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;   //前后方向移动的加速度    向前运动为正
  bf->y = rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;   //左右方向移动的加速度    向右运动为正
  bf->z = rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;   //起降加速度              向上运动为正
}
#include "GCS_Communication.h"
/***********************************************************
*@brief ：由陀螺仪等数据计算得到四元数
*************************************************************/
static void imu_mahony_AHRS_update(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   bool useMag,float dt)
{
    static float acce_inte_x = 0.0f,  acce_inte_y = 0.0f, acce_inte_z = 0.0f;    //加速度积分误差
   // static float magn_inte_x = 0.0f,  magn_inte_y = 0.0f, magn_inte_z = 0.0f;    //磁力计积分误差
    float vec_err_x=0, vec_err_y=0, vec_err_z=0;



    //计算旋转速率(rad/s)
    const float spin_rate_sq = square(gx) + square(gy) + square(gz);

    //Step 1: Yaw correction
    if (useMag)
    {
//        const float magMagnitudeSq = mx * mx + my * my + mz * mz;
//        float kpMag = DCM_KP_MAG *1;
//
//        if (magMagnitudeSq > 0.01f)
//        {
//            //单位化磁力计测量值
//            const float magRecipNorm = sqrt_reciprocal(magMagnitudeSq);
//            mx *= magRecipNorm;
//            my *= magRecipNorm;
//            mz *= magRecipNorm;
//
//            //计算X\Y方向的磁通，磁北方向磁通
//            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
//            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
//            const float bx = sqrtf(hx * hx + hy * hy);
//
//            //磁力计误差是估计磁北和测量磁北之间的交叉乘积
//            const float ez_ef = -(hy * bx);
//
//            //旋转误差到机体坐标系
//            vec_err_x = rMat[2][0] * ez_ef;
//            vec_err_y = rMat[2][1] * ez_ef;
//            vec_err_z = rMat[2][2] * ez_ef;
//        }
//        else
//        {
//            vec_err_x = 0;
//            vec_err_y = 0;
//            vec_err_z = 0;
//        }
//
//        //累计误差补偿
//        if (DCM_KI_MAG > 0.0f)
//        {
//            //如果旋转速率大于限制值则停止积分
//            if (spin_rate_sq < square(Spin_Rate_Limit*RAD_PER_DEG))
//            {
//                magn_inte_x += DCM_KI_MAG * vec_err_x * dt;
//                magn_inte_y += DCM_KI_MAG * vec_err_y * dt;
//                magn_inte_z += DCM_KI_MAG * vec_err_z * dt;
//
//                gx += magn_inte_x;
//                gy += magn_inte_y;
//                gz += magn_inte_z;
//            }
//        }
//
//        //误差补偿
//        gx += kpMag * vec_err_x;
//        gy += kpMag * vec_err_y;
//        gz += kpMag * vec_err_z;
        //! If magnetometer measurement is available, use it.
        if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) {
            float hx, hy, hz, bx, bz;
            float halfwx=0, halfwy=0, halfwz=0;
            float halfex=0, halfey=0, halfez=0;
            // Normalise magnetometer measurement
            // Will sqrt work better? PX4 system is powerful enough?
            const float recipNorm = sqrt_reciprocal(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Reference direction of Earth's magnetic field
            hx = (mx * rMat[0][0] + my *rMat[0][1] + mz *rMat[0][2]);
            hy = (mx * rMat[1][0] + my *rMat[1][1] + mz *rMat[1][2]);
            hz =  mx * rMat[2][0] + my *rMat[2][1] + mz *rMat[2][2];
            bx = sqrt(hx * hx + hy * hy);
            bz = hz;

            // Estimated direction of magnetic field
            halfwx = (bx * rMat[0][0] + bz * rMat[2][0])*0.5f;
            halfwy = (bx * rMat[0][1] + bz * rMat[2][1])*0.5f;
            halfwz = (bx * rMat[0][2] + bz * rMat[2][2])*0.5f;

            //Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (my * halfwz - mz * halfwy);
            halfey = (mz * halfwx - mx * halfwz);
            halfez = (mx * halfwy - my * halfwx);

            //误差补偿
            gx += 1 * halfex;
            gy += 1 * halfey;
            gz += 1 * halfez;
        }

   }

    //Step 2: Roll and pitch correction
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        float accel_x = ax;
        float accel_y = ay;
        float accel_z = az;
        //单位化加速计测量值
        const float accRecipNorm = sqrt_reciprocal(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        const float acc_norm_l = safe_div(1,accRecipNorm,0);
        accel_x *= accRecipNorm;
        accel_y *= accRecipNorm;
        accel_z *= accRecipNorm;

        //加速计读取的方向与重力加速计方向的差值,用向量叉乘计算
        vec_err_x = (accel_z * rMat[2][1] - accel_y * rMat[2][2]);
        vec_err_y = (accel_x * rMat[2][2] - accel_z * rMat[2][0]);
        vec_err_z = (accel_y * rMat[2][0] - accel_x * rMat[2][1]);


        if(acc_norm_l>1060 || acc_norm_l<900)
        {
            vec_err_x = vec_err_y = vec_err_z = 0;
        }
        //累计误差补偿
        if (DCM_KI_ACC > 0.0f)
        {
            //如果旋转速率大于限制值则停止积分
            if (spin_rate_sq < square(Spin_Rate_Limit*RAD_PER_DEG))
            {
                acce_inte_x += DCM_KI_ACC * vec_err_x * dt;
                acce_inte_y += DCM_KI_ACC * vec_err_y * dt;
                acce_inte_z += DCM_KI_ACC * vec_err_z * dt;

                gx += acce_inte_x;
                gy += acce_inte_y;
                gz += acce_inte_z;
            }
        }

        //误差补偿
        gx += DCM_KP_ACC * vec_err_x;
        gy += DCM_KP_ACC * vec_err_y;
        gz += DCM_KP_ACC * vec_err_z;
    }

    //一阶近似算法,四元数运动学方程的离散化形式和积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = ahrs.q0;
    const float qb = ahrs.q1;
    const float qc = ahrs.q2;
    const float qd = ahrs.q3;
    ahrs.q0 += (-qb * gx - qc * gy - qd * gz);
    ahrs.q1 += ( qa * gx + qc * gz - qd * gy);
    ahrs.q2 += ( qa * gy - qb * gz + qd * gx);
    ahrs.q3 += ( qa * gz + qb * gy - qc * gx);

    //单位化四元数
    const float quatRecipNorm = sqrt_reciprocal(ahrs.q0 * ahrs.q0 + ahrs.q1 * ahrs.q1 + ahrs.q2 * ahrs.q2 + ahrs.q3 * ahrs.q3);
    ahrs.q0 *= quatRecipNorm;
    ahrs.q1 *= quatRecipNorm;
    ahrs.q2 *= quatRecipNorm;
    ahrs.q3 *= quatRecipNorm;

    //计算四元数的旋转矩阵
    imu_compute_rotation_matrix();
}

/***********************************************************
*@brief ：由旋转矩阵求得姿态角
*************************************************************/
static void ahrs_update_euler_angles(attitude_t *attitude)
{
    attitude->roll =  (atan2_approx(rMat[2][1], rMat[2][2]))*DEG_PER_RAD;
    attitude->pitch = ((0.5f * PI) - acos_approx(-rMat[2][0]))*DEG_PER_RAD; //arcsin = 0.5PI - arccos
    attitude->yaw =   (atan2_approx(rMat[1][0], rMat[0][0]))*DEG_PER_RAD;

    imuAttitudeYaw = attitude->yaw;

    if (attitude->yaw < 0.0f)   //转换位0~360
        attitude->yaw += 360.0f;

    _attitude_angle.pitch = attitude->pitch;
    _attitude_angle.roll  = attitude->roll;
    _attitude_angle.yaw   = attitude->yaw;
}

attitude_t att;
/**********************************************************************
  * @brief  四元数和姿态解角的计算
  * @param  角速度数据
  * @param  保存姿态角数据的结构体
  * @param  计算步长
**********************************************************************/
void ahrs_update_attitude(const sensorData_t *sensorData, state_t *state, float dt)
{

    Vector3f gyro  = sensorData->gyro;
    Vector3f acce  = sensorData->acce;
    Vector3f magn  = sensorData->magn;

    //角速度单位由度转为弧度
    gyro.x = gyro.x * RAD_PER_DEG;
    gyro.y = gyro.y * RAD_PER_DEG;
    gyro.z = gyro.z * RAD_PER_DEG;

    //计算四元数和旋转矩阵
    imu_mahony_AHRS_update(gyro.x, gyro.y, gyro.z,
                           acce.x, acce.y, acce.z,
                           magn.x, magn.y, magn.z,
                           0,dt);
    //计算欧拉角
    ahrs_update_euler_angles(&state->attitude);
    /* 发送紧急消息到消息队列中 rt_mq_urgent */
    rt_mq_urgent(&ahrsDataQueue,&state->attitude,sizeof(attitude_t));

//    static float q[4]={1,0,0,0};
//    EKF_AHRS_update(gyro.x, gyro.y, gyro.z,
//                    acce.x, acce.y, acce.z,
//                    0, 0, 0,q);
//    att.roll  = ((0.5f * PI) - acos_approx(-( 2.0f * (q[1]*q[3] + -q[0]*q[2]))))*DEG_PER_RAD; //arcsin = 0.5PI - arccos
//    att.pitch = (atan2_approx((2.0f * (q[2]*q[3] - -q[0]*q[1])),(1.0f - 2.0f * q[1]*q[1] - 2.0f * q[2]*q[2])))*DEG_PER_RAD;
//    att.yaw   = (atan2_approx((2.0f * (q[1]*q[2] - -q[0]*q[3])),(1.0f - 2.0f * q[2]*q[2] - 2.0f * q[3]*q[3])))*DEG_PER_RAD;
}



