/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-28     CGY       the first version
 */
#include "DL_InertialSensor.h"
#include "DL_InertialSensor_icm20602.h"
/* 消息队列控制块 */
static struct rt_messagequeue gyroDataQueue;
static char msgGyro_pool[1024];
/* 消息队列控制块 */
static struct rt_messagequeue acceDataQueue;
static char msgAcce_pool[1024];

int imu_acce_gyro_DataQueue_init(void)
{
    /* 初始化消息队列 */
    rt_mq_init(&gyroDataQueue, "mqGyro",
               &msgGyro_pool[0],        /* 内存池指向msg_pool */
               sizeof(Vector3f),        /* 每个消息的大小是字节* */
               sizeof(msgGyro_pool),    /* 内存池的大小是msg_pool的大小 */
               RT_IPC_FLAG_PRIO);       /* 如果有多个线程等待，按照FIFO的方法分配消息 */
    /* 初始化消息队列 */
    rt_mq_init(&acceDataQueue, "mqAcce", &msgAcce_pool[0],sizeof(Vector3f), sizeof(msgAcce_pool), RT_IPC_FLAG_FIFO);
    return RT_EOK;
}
INIT_APP_EXPORT(imu_acce_gyro_DataQueue_init);//自动加入初始化

/*从队列读取陀螺数据*/
bool sensorsReadGyro(Vector3f *gyro)  //为原始数据经第一次低通滤波后的角速度数据 deg/s
{
    /* 从消息队列中接收消息 */
    return 0;//(rt_mq_recv(&gyroDataQueue, gyro, sizeof(Vector3f), 0)== RT_EOK);
}

Vector3f _imu_gyro;
/*从队列读取陀螺数据*/
Vector3f get_imu_sensorsGyro(void)  //为原始数据经第一次低通滤波后的角速度数据 deg/s
{
    return _imu_gyro;
}

Vector3f _imu_raw_gyro;
/*从队列读取陀螺数据*/
Vector3f get_raw_imu_sensorsGyro(void)  //为原始数据经第一次低通滤波后的角速度数据 deg/s
{
    return _imu_raw_gyro;
}

Vector3f _imu_acce;
/*从队列读取加速计数据*/
Vector3f get_imu_sensorsAcce(void)  //为原始数据经第一次低通滤波后的角速度数据 deg/s
{
    return _imu_acce;
}

bool sensorsReadAcce(Vector3f *acce)  //为原始数据经第一次低通滤波后的加速度数据 cm/ss
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&acceDataQueue, acce, sizeof(Vector3f), 0)== RT_EOK);
}

int get_gyro_queue_entry()
{
    /* 队列中已有的消息数 */
    return gyroDataQueue.entry;
}
int get_acce_queue_entry()
{
    /* 队列中已有的消息数 */
    return acceDataQueue.entry;
}


/**************************************imu传感器的角速度数据读取及处理********************************************/
Vector3i16 gyro_raw_adc[MAX_IMU_NUM];    //陀螺仪原始ADC数据
Vector3i16 gyro_adc;                     //转换单位为角度每秒的数据
Vector3f   gyro_deg;                     //转换单位为角度每秒的数据
gyroCalibration_t  gyroCalibration[MAX_IMU_NUM];
IMU_state imu={0,{false},{false},{false},{false}};

float length_V3(Vector3f Vector3)
{
   float vec =  Vector3.x + Vector3.y + Vector3.z;
   return vec;
}
int _instance=0;
int get_gyro_instance(void)
{
    return  _instance;
}
/*********************************************************
*@brief   传感器的角速度数据读取及处理
*@param[in] instance==0时，就是主传感器。
*********************************************************/
void gyro_data_update(Vector3f *gyro,uint8_t instance)
{
    static float alpha=0;
    static uint8_t init=0,i=0,gyro_instance=0xff;
    if (init == 0) {
        for (i = 0; i < MAX_IMU_NUM; ++i) {
            if(imu.gyro_healthy[i] == true)
                gyro_instance = i;
        }
        init = 1;
        _instance =  gyro_instance;
        alpha = calc_lowpass_alpha(0.002f,20);
        Set_Gyro_Sum_Cnt(500,gyro_instance);
    }
    //读取原始数据,注册了哪一个传感器，哪一个的读取函数才会执行。
    icm20602_get_gyro_raw(&gyro_raw_adc[instance],instance);
    icm20689_get_gyro_raw(&gyro_raw_adc[instance],instance);
    icm42605_get_gyro_raw(&gyro_raw_adc[instance],instance);

    for(uint8_t i = 0; i < MAX_IMU_NUM; ++i)
    {
        //陀螺仪校准
        if(!gyroIsCalibrationComplete(i))
        {
            Gyro_Calibration(gyro_raw_adc[i],i);
            gyro_deg.x = 0.0f;
            gyro_deg.y = 0.0f;
            gyro_deg.z = 0.0f;
            return;
        }
    }
    //计算gyro_adc值，原始数据减去零偏。
    gyro_adc.x = (gyro_raw_adc[gyro_instance].x - gyroCalibration[gyro_instance].Zero.x); //俯仰动作时输出的角速度,逆时针旋转为正
    gyro_adc.y = (gyro_raw_adc[gyro_instance].y - gyroCalibration[gyro_instance].Zero.y); //横滚动作时输出的角速度,逆时针旋转为正
    gyro_adc.z = (gyro_raw_adc[gyro_instance].z - gyroCalibration[gyro_instance].Zero.z); //偏航动作时输出的角速度,逆时针旋转为正

    for(uint8_t i = 0; i < 3; ++i)
    {
        gyro_deg.axis[i] += alpha *((float)gyro_adc.axis[i]*GYRO_SCALE - gyro_deg.axis[i]);
    }

    /* 发送紧急消息到消息队列中 rt_mq_urgent */
    //把最新的数据发送到队列的链头,其他慢的线程获取数据时可获得最新数据。
    //rt_mq_urgent(&gyroDataQueue,&gyro_deg,sizeof(Vector3f));
    _imu_gyro = gyro_deg;   //把处理好的角速度数据发布出去

    *gyro = gyro_deg;//返回一个健康imu传感器的数据
}

//设置陀螺仪求和次数
void Set_Gyro_Sum_Cnt(uint16_t calibrationCyclesRequired,uint8_t instance)
{
    gyroCalibration[instance].cycleCount = calibrationCyclesRequired;
}
//陀螺仪校准计数初始化，刚赋初值时返回true，当计数器减少时返回false
bool gyroCalibration_cnt_init(uint8_t instance)
{
    return gyroCalibration[instance].cycleCount == Calibration_Gyro_Cycle;
}
//陀螺仪是否校准成功,当计数器递减到零时,函数返回true
bool gyroIsCalibrationComplete(uint8_t instance)
{
    return gyroCalibration[instance].cycleCount == 0;
}
//陀螺仪数据求和是否成功,计数器递减到1时,函数返回true
bool Gyro_Sum_is_ok(uint8_t instance)
{
    return gyroCalibration[instance].cycleCount == 1;
}

void Gyro_Calibration(Vector3i16 gyroADCsample,uint8_t instance)
{
    for(int i = 0; i < 3; i++)
    {             //复位数据
        if(gyroCalibration_cnt_init(instance))
        {
            gyroCalibration[instance].Sum.axis[i] = 0;
            devClear(&gyroCalibration[instance].var[i]);
        }
        //陀螺数据累加
        gyroCalibration[instance].Sum.axis[i] += gyroADCsample.axis[i];
        devPush(&gyroCalibration[instance].var[i], gyroADCsample.axis[i]);
        //复位零偏
        gyroCalibration[instance].Zero.axis[i] = 0;
        if(Gyro_Sum_is_ok(instance))
        {
            const float stddev=devStandardDeviation(&gyroCalibration[instance].var[i]);
            //检测方差值是否大于陀螺仪受到移动的阈值
            //如果大于设定阈值则返回重新校准
            if((stddev > 10)||(stddev == 0))
            {
                Set_Gyro_Sum_Cnt(Calibration_Gyro_Cycle,instance);
                //send_string("Gyro_Calibration_FALSE!");
                return;
            }
            //校准完成
            gyroCalibration[instance].Zero.axis[i] = (gyroCalibration[instance].Sum.axis[i]) / Calibration_Gyro_Cycle;
            //if(i==2)
               // send_string("Gyro_Calibration_OK!");
        }

    }
    gyroCalibration[instance].cycleCount--;
}


/**************************************imu传感器的加速度数据读取及处理********************************************/
Vector3i16 acce_raw_adc[MAX_IMU_NUM];//陀螺仪原始ADC数据
Vector3f   acce_adc;        //转换单位为角度每秒的数据
Vector3f   acce_cm_ss;      //转换单位为角度每秒的数据
acceCalibration_t  acceCalibration[MAX_IMU_NUM];
/*****************************************************
*@brief   传感器的加速度数据读取及处理
*@param[in] instance==0时，就是主传感器。
*****************************************************/
void acce_data_update(Vector3f *acce,uint8_t instance)
{
    static float alpha=0;
    static uint8_t init=0,i=0,acce_instance=0xff;
    if (init == 0) {
        for (i = 0; i < 3; ++i) {
            if(imu.gyro_healthy[i]==true)
                acce_instance = i;
        }
        init = 1;
        alpha = calc_lowpass_alpha(0.002f,15);
        Set_Acce_Sum_Cnt(500,acce_instance);
    }

    //读取原始数据,注册了哪一个传感器，哪一个的读取函数才会执行。
    icm20602_get_acce_raw(&acce_raw_adc[instance],instance);
    icm20689_get_acce_raw(&acce_raw_adc[instance],instance);
    icm42605_get_acce_raw(&acce_raw_adc[instance],instance);

    for(uint8_t i = 0; i < MAX_IMU_NUM; ++i)
    {
        if(!acceIsCalibrationComplete(i))
        {
            Acce_Calibration(acce_raw_adc[i],i);
            return ;
        }
    }

    acce_adc.x = (acce_raw_adc[acce_instance].x - acceCalibration[acce_instance].Zero.x);    //横滚加速度,向右运动为正
    acce_adc.y = (acce_raw_adc[acce_instance].y - acceCalibration[acce_instance].Zero.y);    //俯仰加速度,向前运动为正
    acce_adc.z = (acce_raw_adc[acce_instance].z - acceCalibration[acce_instance].Zero.z);    //起降加速度,向上运动为正

    for(uint8_t i = 0; i < 3; ++i)
    {
        acce_cm_ss.axis[i] += alpha*(acce_adc.axis[i] * ACC_1G_ADC - acce_cm_ss.axis[i]) ;
    }

    ///* 发送紧急消息到消息队列中 rt_mq_urgent */
    //rt_mq_urgent(&acceDataQueue,&acce_g, sizeof(Vector3f));
    _imu_acce = acce_cm_ss; //把处理好的角速度数据发布出去

    *acce = acce_cm_ss;//返回一个健康imu传感器的数据
}

//设置加速度求和次数
void Set_Acce_Sum_Cnt(uint16_t calibrationCyclesRequired,uint8_t instance)
{
    acceCalibration[instance].cycleCount = calibrationCyclesRequired;
}
//加速度校准计数初始化，刚赋初值时返回true，当计数器减少时返回false
static bool acceCalibration_cnt_init(uint8_t instance)
{
    return acceCalibration[instance].cycleCount == Calibration_Acce_Cycle;
}
//加速度是否校准成功。当计数器递减到零时，函数返回true
bool acceIsCalibrationComplete(uint8_t instance)
{
    return acceCalibration[instance].cycleCount == 0;
}
//加速度数据求和是否成功。计数器递减到1时，函数返回true
static bool Acce_Sum_is_ok(uint8_t instance)
{
    return acceCalibration[instance].cycleCount == 1;
}

void Acce_Calibration(Vector3i16 acceADCsample,uint8_t instance)
{
    for(int i = 0; i < 3; i++)
    {       //复位数据
        if(acceCalibration_cnt_init(instance))
        {
           acceCalibration[instance].Sum.axis[i] = 0;
           devClear(&acceCalibration[instance].var[i]);
        }
        //加速度数据累加
        acceCalibration[instance].Sum.axis[i] += acceADCsample.axis[i];
        devPush(&acceCalibration[instance].var[i], acceADCsample.axis[i]);
        //复位零偏
        acceCalibration[instance].Zero.axis[i] = 0;
        if(Acce_Sum_is_ok(instance))
        {
            const float stddev = devStandardDeviation(&acceCalibration[instance].var[i]);
            const float acce_Zero = (acceCalibration[instance].Sum.axis[2]  / Calibration_Acce_Cycle);
            //检测方差值是否大于加速度受到移动的阈值
            //如果大于设定阈值则返回重新校准
            if ((stddev > 20)||(stddev == 0)||acce_Zero<-4596||acce_Zero>-3596)
            {
                Set_Acce_Sum_Cnt(Calibration_Acce_Cycle,instance);
                //send_string("Acce_Calibration_FALSE!");
                return;
            }
            //校准完成
            acceCalibration[instance].Zero.axis[0] = (acceCalibration[instance].Sum.axis[0]) / Calibration_Acce_Cycle;
            acceCalibration[instance].Zero.axis[1] = (acceCalibration[instance].Sum.axis[1]) / Calibration_Acce_Cycle;
            acceCalibration[instance].Zero.axis[2] = (acceCalibration[instance].Sum.axis[2]  / Calibration_Acce_Cycle)+4096;

//            if(i==2)
//            {
//                Parame.set.AcceZero[0]=acceCalibration.Zero.axis[0];
//                Parame.set.AcceZero[1]=acceCalibration.Zero.axis[1];
//                Parame.set.AcceZero[2]=acceCalibration.Zero.axis[2];
//                data_save_to_Flsh();
//                send_string("Acce_Calibration_OK!");
//            }

        }

    }

    acceCalibration[instance].cycleCount--;
}




//调整传感器坐标系 使传感器坐标和飞控坐标对齐
void applySensorAlignment(int16_t * dest, int16_t * src, uint8_t rotation)
{
    const int16_t x = src[0];
    const int16_t y = src[1];
    const int16_t z = src[2];

    switch (rotation) {
    default:
    case 0://不旋转
        dest[0] = x;
        dest[1] = y;
        dest[2] = z;
        break;
    case 1://顺时针旋转90度
        dest[0] =  y;
        dest[1] = -x;
        dest[2] =  z;
        break;
    case 2://顺时针旋转180度
        dest[0] = -x;
        dest[1] = -y;
        dest[2] =  z;
        break;
    case 3://顺时针旋转270度
        dest[0] = -y;
        dest[1] =  x;
        dest[2] =  z;
        break;
    case 4://反向0度
        dest[0] = -x;
        dest[1] =  y;
        dest[2] = -z;
        break;
    case 5://反向后顺时针旋转90度
        dest[0] =  y;
        dest[1] =  x;
        dest[2] = -z;
        break;
    case 6://反向后顺时针旋转180度
        dest[0] =  x;
        dest[1] = -y;
        dest[2] = -z;
        break;
    case 7://反向后顺时针旋转270度
        dest[0] = -y;
        dest[1] = -x;
        dest[2] = -z;
        break;
    }

}



