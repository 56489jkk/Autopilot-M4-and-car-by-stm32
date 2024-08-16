/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-07     CGY       the first version
 */
#include "loco_config.h"
#include "DL_T265.h"
sensors_state_t state;
int32_t last_t265_update_ms;

bool get_t265_health()
{
    if(rt_tick_get() - last_t265_update_ms > 1000){
        return  false;
    }
    else  return  true;
}
_Vio_Rx t265_rx;

void t265_data_Receive(uint8_t data)
{
    static uint8_t data_len = 0;
    static uint8_t state = 0;

    if(state==0&&data==0x55)    //
    {
        state=1;
        t265_rx.buf[0]=data;
    }
    else if(state==1&&data==0xAA)   //
    {
        state=2;
        t265_rx.buf[1]=data;
        data_len = 63;
        t265_rx.len=0;
    }
    else if(state==2&&data_len>0)
    {
        data_len--;
        t265_rx.buf[2+t265_rx.len++]=data;
        if(data_len==0)
        state = 3;
    }
    else if(state==3 && t265_rx.buf[63]==0xAA)
    {
        state=0;
        T265_data_deal(t265_rx);
    }
    else
    {
        state=0;
    }
}

int rudder_flag=0;
int postion_flag=0;
int into_postion=0;
vision_estimate_t  t265_estimate,t265_target;
bool _t265_landFlag = false;
/**********************************************************************************************************
*函 数 名: T265_datadeal
*功能说明: 接收视觉里程计数据
*形    参: 无
*返 回 值: Position
**********************************************************************************************************/
void T265_data_deal(_Vio_Rx rx)
{
    //估计状态定义
    float_union  position_x,position_y,position_z;                    //估计位置
    float_union  velocity_x,velocity_y,velocity_z;                    //估计速度
    //float_union  Quaternion0,Quaternion1,Quaternion2,Quaternion3;   //估计姿态
    float_union  rpy_yaw;                                             //估计姿态

    unsigned char CheckSum=0;
    if(rx.buf[0]==0x55 && rx.buf[1]==0xAA && rx.buf[63]==0xAA)
    {
        for(unsigned char i=0;i<62;i++)
        {
            CheckSum += rx.buf[i];
        }

        if(rx.buf[2] == 0x10 &&  rx.buf[64] == CheckSum )
        {
            //X轴位置数据
            position_x.cv[0] = rx.buf[3];
            position_x.cv[1] = rx.buf[4];
            position_x.cv[2] = rx.buf[5];
            position_x.cv[3] = rx.buf[6];
            //Y轴位置数据
            position_y.cv[0] = rx.buf[7];
            position_y.cv[1] = rx.buf[8];
            position_y.cv[2] = rx.buf[9];
            position_y.cv[3] = rx.buf[10];
            //Z轴位置数据
            position_z.cv[0] = rx.buf[11];
            position_z.cv[1] = rx.buf[12];
            position_z.cv[2] = rx.buf[13];
            position_z.cv[3] = rx.buf[14];
            //X轴速度数据
            velocity_x.cv[0] = rx.buf[15];
            velocity_x.cv[1] = rx.buf[16];
            velocity_x.cv[2] = rx.buf[17];
            velocity_x.cv[3] = rx.buf[18];
            //Y轴速度数据
            velocity_y.cv[0] = rx.buf[19];
            velocity_y.cv[1] = rx.buf[20];
            velocity_y.cv[2] = rx.buf[21];
            velocity_y.cv[3] = rx.buf[22];
            //Z轴速度数据
            velocity_z.cv[0] = rx.buf[23];
            velocity_z.cv[1] = rx.buf[24];
            velocity_z.cv[2] = rx.buf[25];
            velocity_z.cv[3] = rx.buf[26];
            //视觉里程计的姿态数据
            rpy_yaw.cv[0] = rx.buf[27];
            rpy_yaw.cv[1] = rx.buf[28];
            rpy_yaw.cv[2] = rx.buf[29];
            rpy_yaw.cv[3] = rx.buf[30];

            t265_estimate.pos_x =  position_y.fvalue*100.0f;
            t265_estimate.pos_y = -position_x.fvalue*100.0f;
            t265_estimate.pos_z =  position_z.fvalue*100.0f;

            t265_estimate.vel_x =  velocity_y.fvalue*100.0f;  //向右为正方向
            t265_estimate.vel_y = -velocity_x.fvalue*100.0f;  //向前为正方向
            t265_estimate.vel_z =  velocity_z.fvalue*100.0f;  //向上为正方向

            t265_estimate.yaw = -rpy_yaw.fvalue*180.0f/PI;

            if (t265_estimate.yaw < 0.0f)   //转换位0~360  逆时针增大和机体yaw一致
                t265_estimate.yaw += 360.0f;


            //期望的位置数据
            float_union  reference_posx,reference_posy,reference_posz;      //参考航点
            reference_posx.cv[0] = rx.buf[44];
            reference_posx.cv[1] = rx.buf[45];
            reference_posx.cv[2] = rx.buf[46];
            reference_posx.cv[3] = rx.buf[47];

            reference_posy.cv[0] = rx.buf[48];
            reference_posy.cv[1] = rx.buf[49];
            reference_posy.cv[2] = rx.buf[50];
            reference_posy.cv[3] = rx.buf[51];

            reference_posz.cv[0] = rx.buf[52];
            reference_posz.cv[1] = rx.buf[53];
            reference_posz.cv[2] = rx.buf[54];
            reference_posz.cv[3] = rx.buf[55];

            rudder_flag = rx.buf[56]; //0:都不打开、 1：打开第一个
            postion_flag= rx.buf[57]; //0:激光笔关闭、 1：激光笔打开

            t265_target.pos_x = reference_posx.fvalue*100;
            t265_target.pos_y = reference_posy.fvalue*100;
            t265_target.pos_z = reference_posz.fvalue*100;


            state.arm_power =  ArmPowerON;
            // 自主路径巡航完成后终点的自主降落
            if(rx.buf[62]==0xA5)                //树莓派发送降落指令
            {
                _t265_landFlag = true;
            }
            else {
                _t265_landFlag = false;
            }

            //if (!is_zero(velocity_x.fvalue)||!is_zero(velocity_y.fvalue)||!is_zero(velocity_z.fvalue))
            {
                last_t265_update_ms = rt_tick_get();
            }
        }
        //switchs.arm_on：树莓派是否上电标志位哦
        else if(rx.buf[2]==0xFF)//
        {
            state.arm_power = ArmPowerOFF;
        }

    }
}
bool is_t265_landFlag()
{
    return _t265_landFlag;
}

/**********************************************************************************************************
*函 数 名: SendTakeOffFlag
*功能说明: 发送起飞标志给树莓派
**********************************************************************************************************/
void send_take_off_flag_to_cv(void)
{
    uint8_t dataToARM[11] = "Departures\n";
    rt_device_t dev = serial_device_find(SerialProtocol_Vision);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}

/**********************************************************************************************************
*函 数 名: SendT265StartFlag
*功能说明: 发送启动双目视觉命令给树莓派
**********************************************************************************************************/
void send_cmd_start_t265(void)
{
    uint8_t dataToARM[9] = "Start265\n";
    rt_device_t dev = serial_device_find(SerialProtocol_Vision);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}

/**********************************************************************************************************
*函 数 名: RestartT265
*功能说明: 发送重启双目视觉命令给树莓派
**********************************************************************************************************/
void send_cmd_restart_t265(void)
{
    uint8_t dataToARM[11] = "Refresh265\n";
    rt_device_t dev = serial_device_find(SerialProtocol_Vision);
    if(dev ) rt_device_write(dev,0,dataToARM,sizeof(dataToARM));
}

