/**
 * @file shoot.h
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
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
	uint8_t pull_bullets_state;//记录上一次开关的状态
	uint8_t fric_flag;//记录摩擦轮是否有速度
};


extern struct Struct_SHOOT_Manage_Object shoot_control;
extern void shoot_init(struct Struct_SHOOT_Manage_Object *shoot);
extern void shoot_task(struct Struct_SHOOT_Manage_Object *shoot);
extern const RC_ctrl_t *local_rc_ctrl;
#endif
