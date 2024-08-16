/**
 * @file pid.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief PID������ݽṹ����������
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _PID_H
#define _PID_H

#include <stdint.h>

/**
  * @brief PID �������ṹ��
  *
  */
typedef struct {
	float set_point; /**< @brief Ŀ��ֵ */
	float kp;        /**< @brief �������� */
	float ki;        /**< @brief �������� */
	float kd;        /**< @brief ΢������ */
	
	float previous_0_err; /**< @brief �ϴ���� */
	float previous_1_err; /**< @brief ���ϴ���� */
	
	float output; /**< @brief PID��� */
}PID_ControllerTypeDef;



/**
 * @brief PID���Ƹ���
 * @param self PID����������ָ��
 * @param actual ��ǰ��ʵ��ֵ
 * @param �����ϴθ��µ�ʱ����
 * @retval None.
 */
void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta);



/**
 * @brief ��ʼ��PID������
 * @param self Ҫ��ʼ����PID������ָ��
 * @param kp ��������
 * @param ki ��������
 * @param kd ΢������
 * @retval None.
 */
void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);

#endif
