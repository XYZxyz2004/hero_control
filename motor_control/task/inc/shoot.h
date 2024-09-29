/**
 * @file shoot.h
 * @author xyz
 * @brief ��ɷ�������ĳ�ʼ�������岦���Ͳ�ͬ�ٷ���
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#ifndef SHOOT_H
#define SHOOT_H
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_pid.h"
#include "bsp_motor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "remote_control.h"

struct Struct_SHOOT_Manage_Object
{
	struct Struct_MOTOR_Manage_Object pull_bullets_motor ;
	struct Struct_MOTOR_Manage_Object fric_wheel_l_motor ;
	struct Struct_MOTOR_Manage_Object fric_wheel_r_motor ;
	uint8_t pull_bullets_state;//��¼��һ�ο��ص�״̬
	uint8_t fric_flag;//��¼Ħ�����Ƿ����ٶ�
};


extern struct Struct_SHOOT_Manage_Object shoot_control;
extern void shoot_init(struct Struct_SHOOT_Manage_Object *shoot);
extern void shoot_task(struct Struct_SHOOT_Manage_Object *shoot);
extern const RC_ctrl_t *local_rc_ctrl;
#endif
