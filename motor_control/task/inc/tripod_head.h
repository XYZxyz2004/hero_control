/**
 * @file tripod_head.c
 * @author xyz
 * @brief �����̨������ƫ��
 * @version 1.1
 * @date 2024-09-29
 * @copyright Transistor BUAA
 */
#ifndef TRIPOD_HEAD_H
#define TRIPOD_HEAD_H
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

struct Struct_TRIPOD_Manage_Object
{

	struct Struct_MOTOR_Manage_Object yaw_motor ;
	struct Struct_MOTOR_Manage_Object pitch_motor ;
  float pitch_down_location_limit; //pitch����λ
	float pitch_up_location_limit;  //pitch����λ
	float pitch_start_location;   //pitch�ϵ�λ��
	float yaw_left_location_limit;    //yaw����λ
	float yaw_right_location_limit;   //yaw����λ
	float yaw_start_location;    //yaw�ϵ�λ��
};

extern struct Struct_TRIPOD_Manage_Object tripod_head_control;
extern void tripod_init(struct Struct_TRIPOD_Manage_Object *tripod);
extern void tripod_task(struct Struct_TRIPOD_Manage_Object *tripod);
extern const RC_ctrl_t *local_rc_ctrl;
#endif