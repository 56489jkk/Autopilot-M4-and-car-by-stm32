/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-04-11     Administrator       the first version
 */
#include "DL_LinkTrack_P_BS2.h"

/******************************UWB测试部分******************************/
static uint32_t last_uwb_update_ms = 0;
bool get_uwb_health()
{
    if(rt_tick_get() - last_uwb_update_ms > 100){
        return  false;
    }
    else  return  true;
}

uint8_t verifyCheckSum(uint8_t *data, int32_t length)
{
    uint8_t sum = 0;
    for(int32_t i=0;i<length-1;++i){
        sum += data[i];
    }
    return sum == data[length-1];
}

uwb_data_t  UWB;
void uwb_data_receive(uint8_t data)
{
    static uint8_t data_len = 0;
    static uint8_t state = 0;
    if(state==0&&data==0x55)    //
    {
        state=1;
        UWB.data[0]=data;
    }
    else if(state==1&&data==0x01)   //
    {
        state=2;
        data_len = 126;
        UWB.data[1]=data;
        UWB.dataLen = 2;
    }
    else if(state==2&&data_len>0)
    {
        data_len--;
        UWB.data[UWB.dataLen++]=data;
        if(data_len==0){
            state=0;
            bool verifyCheck = verifyCheckSum(UWB.data,128);
            if (verifyCheck == 1) {
                UWB.raw_px = (int32_t)(UWB.data[4] << 8 | UWB.data[5] << 16 | UWB.data[6] << 24) / 256;
                UWB.raw_py = (int32_t)(UWB.data[7] << 8 | UWB.data[8] << 16 | UWB.data[9] << 24) / 256;
                UWB.raw_pz = (int32_t)(UWB.data[10] << 8 | UWB.data[11] << 16 | UWB.data[12] << 24) / 256;
                UWB.raw_vx = (int32_t)(UWB.data[13] << 8 | UWB.data[14] << 16 | UWB.data[15] << 24) / 256;
                UWB.raw_vy = (int32_t)(UWB.data[16] << 8 | UWB.data[17] << 16 | UWB.data[18] << 24) / 256;
                UWB.raw_vz = (int32_t)(UWB.data[19] << 8 | UWB.data[20] << 16 | UWB.data[21] << 24) / 256;
                UWB.distance[0] = (int32_t)(UWB.data[22] << 8 | UWB.data[23] << 16 | UWB.data[24] << 24) / 256;
                UWB.distance[1] = (int32_t)(UWB.data[25] << 8 | UWB.data[26] << 16 | UWB.data[27] << 24) / 256;
                UWB.distance[2] = (int32_t)(UWB.data[28] << 8 | UWB.data[29] << 16 | UWB.data[30] << 24) / 256;
                UWB.distance[3] = (int32_t)(UWB.data[31] << 8 | UWB.data[32] << 16 | UWB.data[33] << 24) / 256;
//                UWB.distance[4] = (int32_t)(UWB.data[34] << 8 | UWB.data[35] << 16 | UWB.data[36] << 24) / 256;
//                UWB.distance[5] = (int32_t)(UWB.data[37] << 8 | UWB.data[38] << 16 | UWB.data[39] << 24) / 256;
//                UWB.distance[6] = (int32_t)(UWB.data[40] << 8 | UWB.data[41] << 16 | UWB.data[42] << 24) / 256;
//                UWB.distance[7] = (int32_t)(UWB.data[43] << 8 | UWB.data[44] << 16 | UWB.data[45] << 24) / 256;
                UWB.sonar_time = (uint32_t)((UWB.data[115] << 24) | (UWB.data[114] << 16) | (UWB.data[113] << 8) | UWB.data[112]);
                UWB.eop.x = (float)UWB.data[117]/100.0f;
                UWB.eop.y = (float)UWB.data[118]/100.0f;
                UWB.eop.z = (float)UWB.data[119]/100.0f;
                UWB.sonar_vol = ((rt_uint16_t)(UWB.data[121]<<8)|UWB.data[120]);

                UWB.pos.x =  UWB.raw_px/1000.0f;
                UWB.pos.y =  UWB.raw_py/1000.0f;
                UWB.pos.z =  UWB.raw_pz/1000.0f;
                UWB.vel.x =  UWB.raw_vx/1000.0f;
                UWB.vel.y =  UWB.raw_vy/1000.0f;
                UWB.vel.z =  UWB.raw_vz/1000.0f;
                last_uwb_update_ms = rt_tick_get();
            }
        }
    }
    else
    {
        state=0;
    }
}

void uwb_data_dma_receive(uint8_t *data_buf)
{
    bool verifyCheck = verifyCheckSum(data_buf,128);
    if (data_buf[0]==0x55 && verifyCheck == 1) {
        UWB.raw_px = (int32_t)(data_buf[4] << 8 | data_buf[5] << 16 | data_buf[6] << 24) / 256;
        UWB.raw_py = (int32_t)(data_buf[7] << 8 | data_buf[8] << 16 | data_buf[9] << 24) / 256;
        UWB.raw_pz = (int32_t)(data_buf[10] << 8 | data_buf[11] << 16 | data_buf[12] << 24) / 256;
        UWB.raw_vx = (int32_t)(data_buf[13] << 8 | data_buf[14] << 16 | data_buf[15] << 24) / 256;
        UWB.raw_vy = (int32_t)(data_buf[16] << 8 | data_buf[17] << 16 | data_buf[18] << 24) / 256;
        UWB.raw_vz = (int32_t)(data_buf[19] << 8 | data_buf[20] << 16 | data_buf[21] << 24) / 256;
        UWB.distance[0] = (int32_t)(data_buf[22] << 8 | data_buf[23] << 16 | data_buf[24] << 24) / 256;
        UWB.distance[1] = (int32_t)(data_buf[25] << 8 | data_buf[26] << 16 | data_buf[27] << 24) / 256;
        UWB.distance[2] = (int32_t)(data_buf[28] << 8 | data_buf[29] << 16 | data_buf[30] << 24) / 256;
        UWB.distance[3] = (int32_t)(data_buf[31] << 8 | data_buf[32] << 16 | data_buf[33] << 24) / 256;
        //UWB.distance[4] = (int32_t)(data_buf[34] << 8 | data_buf[35] << 16 | data_buf[36] << 24) / 256;
        //UWB.distance[5] = (int32_t)(data_buf[37] << 8 | data_buf[38] << 16 | data_buf[39] << 24) / 256;
        //UWB.distance[6] = (int32_t)(data_buf[40] << 8 | data_buf[41] << 16 | data_buf[42] << 24) / 256;
        //UWB.distance[7] = (int32_t)(data_buf[43] << 8 | data_buf[44] << 16 | data_buf[45] << 24) / 256;
        UWB.sonar_time = (uint32_t)((data_buf[115] << 24) | (data_buf[114] << 16) | (data_buf[113] << 8) | data_buf[112]);
        UWB.eop.x = (float)data_buf[117]/100.0f;
        UWB.eop.y = (float)data_buf[118]/100.0f;
        UWB.eop.z = (float)data_buf[119]/100.0f;
        UWB.sonar_vol = ((rt_uint16_t)(data_buf[121]<<8)|data_buf[120]);
        UWB.pos.x =  UWB.raw_px/1000.0f;
        UWB.pos.y =  UWB.raw_py/1000.0f;
        UWB.pos.z =  UWB.raw_pz/1000.0f;
        UWB.vel.x =  UWB.raw_vx/1000.0f;
        UWB.vel.y =  UWB.raw_vy/1000.0f;
        UWB.vel.z =  UWB.raw_vz/1000.0f;
        UWB.dataFre = 1000/(rt_tick_get()-last_uwb_update_ms);  //Hz
        last_uwb_update_ms = rt_tick_get();
    };
}
