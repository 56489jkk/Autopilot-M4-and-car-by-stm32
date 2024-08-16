/**
 * @file encoder_motor.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief �������������ͷ�ļ�, !!! ���ļ��ѱ��޸����ڱ���������ʾ�� !!!
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"


/* �����ÿת����11������,  ��������AB�������½��ؾ�����������ֵΪ��������4��, ���ٱ�Ϊ90.0��
 * ����������ÿתһȦ����������ֵ�ı� 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_JGB520_PID_KP  63.0f
#define MOTOR_JGB520_PID_KI  2.6f
#define MOTOR_JGB520_PID_KD  2.4f
#define MOTOR_JGB520_RPS_LIMIT 1.5f

/* �����ÿת����11������,  ��������AB�������½��ؾ�����������ֵΪ��������4��, ���ٱ�Ϊ45.0:1��
 * ����������ÿתһȦ����������ֵ�ı� 11.0 * 4.0 * 45.0 = 1980
 */
#define MOTOR_JGB37_TICKS_PER_CIRCLE 1980.0f
#define MOTOR_JGB37_PID_KP  40.0f
#define MOTOR_JGB37_PID_KI  2.0f
#define MOTOR_JGB37_PID_KD  2.0f
#define MOTOR_JGB37_RPS_LIMIT 3.0f

/* �����ÿת����13������,  ��������AB�������½��ؾ�����������ֵΪ��������4��, ���ٱ�Ϊ20.0:1��
 * ����������ÿתһȦ����������ֵ�ı� 13.0 * 4.0 * 20.0 = 1040
 */
#define MOTOR_JGA27_TICKS_PER_CIRCLE 1040.0f
#define MOTOR_JGA27_PID_KP  -36.0f
#define MOTOR_JGA27_PID_KI  -1.0f
#define MOTOR_JGA27_PID_KD  -1.0f
#define MOTOR_JGA27_RPS_LIMIT 6.0f 

/* �����ÿת����11������,  ��������AB�������½��ؾ�����������ֵΪ��������4��, ���ٱ�Ϊ131.0:1��
 * ����������ÿתһȦ����������ֵ�ı� 11.0 * 4.0 * 131.0 = 5764
 */
#define MOTOR_JGB528_TICKS_PER_CIRCLE 5764.0f
#define MOTOR_JGB528_PID_KP  300.0f
#define MOTOR_JGB528_PID_KI  2.0f
#define MOTOR_JGB528_PID_KD  12.0f
#define MOTOR_JGB528_RPS_LIMIT 1.1f 

/* �����ÿת����11������,  ��������AB�������½��ؾ�����������ֵΪ��������4��, ���ٱ�Ϊ90.0��
 * ����������ÿתһȦ����������ֵ�ı� 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_DEFAULT_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_DEFAULT_PID_KP  63.0f
#define MOTOR_DEFAULT_PID_KI  2.6f
#define MOTOR_DEFAULT_PID_KD  2.4f
#define MOTOR_DEFAULT_RPS_LIMIT 1.35f

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

/**
 * @brief �������������ṹ��
*/
struct EncoderMotorObject{
		float current_position;
		int pos_ctrl_count;
    int64_t counter;        /**< @brief �ܼ���ֵ, 64bit ��Ϊ������� */
    int64_t overflow_num;   /**< @brief ������� */
    int32_t ticks_overflow; /**< @brief �������ֵ */
    float tps;              /**< @brief ticks per second ������Ƶ�� */
    float rps;              /**< @brief revolutions per second �����ת�� תÿ�� */
		bool enable_pos_control; /**< @brief �Ƿ���λ�ÿ��� */
    int current_pulse;      /**< @brief ��ǰ�����PWMֵ, �з��Ŷ�Ӧ����ת */
		float speed_i;
    PID_ControllerTypeDef pid_controller; /**< @brief PID ������ */
		PID_ControllerTypeDef pid_pos_controller; 

    /** porting ����ֲӲ���ӿ� **/
    int32_t ticks_per_circle; /**< @brief ����������תһȦ�����ļ�������, ���ݵ��ʵ�������д */
	  float rps_limit;  /**< @brief �����ת�ټ��ޣ�һ���ȡ��100% PWMռ�ձ�ʱ��ת����С��ֵ��ȷ��PID���������ٶȵĿ��� */
    /**
      * @brief ���õ���ٶ�  -1000 ~ 1000
      * @param self �������������
      * @param pulse �µ�PWMֵ, -1000~1000, ��Ӧ����ת0~100%ת��
      * @retval None.
      */
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};


/**
 * @breif ��������������ʼ��
 * @param self �������������ָ��
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);


/**
 * @brief ����������ٶȲ�������
 * @detials
 * @param self �������������
 * @param period ���ε������ϴε��õ�ʱ��������λΪ��
 * @param counter ��������ǰ����ֵ
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);


/**
 * @brief ����������ٶȿ�������
 * @detials ����������ٶ�PID��������,��Ҫ��ʱָ�������PID���Ƹ���
 * @param self �������������
 * @param period ��ǰ���¾����ϴθ��µ�ʱ����(��������), ��λ sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);


/**
 * @brief �������������PID����Ŀ���ٶ�
 * @param rps Ŀ���ٶȣ� ��λתÿ��
 * @retval None.
*/
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

/**
 * @brief �������������PID��������Ŀ��λ��
 * @param rps Ŀ���ٶȣ� ��λתÿ��
 * @retval None.
*/
void encoder_motor_set_target(EncoderMotorObjectTypeDef *self, float target_position);
void Forward(void);
void Stopward(void);
void navigate_to_ball(int x, int y);

#endif

