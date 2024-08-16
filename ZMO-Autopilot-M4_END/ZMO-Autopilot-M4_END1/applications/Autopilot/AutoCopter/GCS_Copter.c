/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-12-18     Administrator       the first version
 */

#include "mavlink.h"
#include "GCS_Communication.h"
#include "GCS_Copter.h"


#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  1
#define MAV_MODE_FLAG_TEST_ENABLED         2
#define MAV_MODE_FLAG_AUTO_ENABLED         4
#define MAV_MODE_FLAG_GUIDED_ENABLED       8
#define MAV_MODE_FLAG_STABILIZE_ENABLED   16
#define MAV_MODE_FLAG_HIL_ENABLED         32
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED  64
#define MAV_MODE_FLAG_SAFETY_ARMED          128
MAV_MODE Copter_base_mode()
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (get_new_mode()) {
    case AUTO_Mode:
    case AUTO_RTL_Mode:
    case RTL_Mode:
    case LOITER_Mode:
    case AVOID_ADSB_Mode:
    case FOLLOW_Mode:
    case GUIDED_Mode:
    case CIRCLE_Mode:
    case POSHOLD_Mode:
    case BRAKE_Mode:
    case SMART_RTL_Mode:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        break;
    default:
        break;
    }

    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (motor.state) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    //rt_kprintf("_base_mode %u  %u\n", _base_mode,_base_mode);
    return (MAV_MODE)_base_mode;
}


void Copter_handle_Message(mavlink_message_t msg) //Copter消息接收
{
//    // for mavlink SET_POSITION_TARGET messages
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE =
//        POSITION_TARGET_TYPEMASK_X_IGNORE |
//        POSITION_TARGET_TYPEMASK_Y_IGNORE |
//        POSITION_TARGET_TYPEMASK_Z_IGNORE;
//
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE =
//        POSITION_TARGET_TYPEMASK_VX_IGNORE |
//        POSITION_TARGET_TYPEMASK_VY_IGNORE |
//        POSITION_TARGET_TYPEMASK_VZ_IGNORE;
//
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE =
//        POSITION_TARGET_TYPEMASK_AX_IGNORE |
//        POSITION_TARGET_TYPEMASK_AY_IGNORE |
//        POSITION_TARGET_TYPEMASK_AZ_IGNORE;
//
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE =
//        POSITION_TARGET_TYPEMASK_YAW_IGNORE;
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE =
//        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
//    uint32_t MAVLINK_SET_POS_TYPE_MASK_FORCE_SET =
//        POSITION_TARGET_TYPEMASK_FORCE_SET;

    switch (msg.msgid) {

        case MAVLINK_MSG_ID_MANUAL_CONTROL:
        {
            mavlink_manual_control_t packet;
            mavlink_msg_manual_control_decode(&msg, &packet);

            break;
        }

        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82 设置姿态期望
        {
            // decode packet
            mavlink_set_attitude_target_t packet;
            mavlink_msg_set_attitude_target_decode(&msg, &packet);

            if (get_new_mode()!=GUIDED_Mode) {
                break;
            }

            break;
        }
        case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84 设置位置速度期望
        {
            // decode packet
            mavlink_set_position_target_local_ned_t packet;
            mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);
            mavlink_set_target = packet;
            break;
        }

        default:
            GCS_MAVLINK_handle_common_message(msg);
            break;
    }// end switch

}


/*
  send PID tuning message
 */
void Copter_send_pid_tuning()
{
//    static const PID_TUNING_AXIS axes[] = {
//        PID_TUNING_ROLL,
//        PID_TUNING_PITCH,
//        PID_TUNING_YAW,
//        PID_TUNING_ACCZ
//    };
//    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
//        if (!(copter.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
//            continue;
//        }
//        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
//            return;
//        }
//        const AP_PIDInfo *pid_info = nullptr;
//        switch (axes[i]) {
//        case PID_TUNING_ROLL:
//            pid_info = &copter.attitude_control->get_rate_roll_pid().get_pid_info();
//            break;
//        case PID_TUNING_PITCH:
//            pid_info = &copter.attitude_control->get_rate_pitch_pid().get_pid_info();
//            break;
//        case PID_TUNING_YAW:
//            pid_info = &copter.attitude_control->get_rate_yaw_pid().get_pid_info();
//            break;
//        case PID_TUNING_ACCZ:
//            pid_info = &copter.pos_control->get_accel_z_pid().get_pid_info();
//            break;
//        default:
//            continue;
//        }
//        if (pid_info != nullptr) {
//            mavlink_msg_pid_tuning_send(chan,
//                                        axes[i],
//                                        pid_info->target,
//                                        pid_info->actual,
//                                        pid_info->FF,
//                                        pid_info->P,
//                                        pid_info->I,
//                                        pid_info->D,
//                                        pid_info->slew_rate,
//                                        pid_info->Dmod);
//        }
//    }
}
