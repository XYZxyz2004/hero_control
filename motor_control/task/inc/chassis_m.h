/**
 * @file shoot.h
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#ifndef CHASSIS_M_H
#define CHASSIS_M_H
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
#include "math.h"

#define rx_chassism 0.278
#define ry_chassism 0.18
#define r_chassism  0.077
#define pi 3.14159

typedef struct
{
	int8_t symbol[4];
	float omega;
} motor_chassism;



struct Struct_CHASSISM_Manage_Object
{
	struct Struct_MOTOR_Manage_Object L1_motor ;
	struct Struct_MOTOR_Manage_Object L2_motor ;
	struct Struct_MOTOR_Manage_Object R1_motor ;
	struct Struct_MOTOR_Manage_Object R2_motor ;
	float vx,vy,w;          //底盘x,y向转速，自转角速度
  float max_speed;         //底盘移动最大速度
	motor_chassism chassis_motor[4];
};

extern void chassism_calculate(int16_t ch1,int16_t ch2,int16_t ch3,struct Struct_CHASSISM_Manage_Object *chassis);
extern void backwardm_calc(struct Struct_CHASSISM_Manage_Object *chassis) ;

extern struct Struct_CHASSISM_Manage_Object chassism_control;
extern void chassism_init(struct Struct_CHASSISM_Manage_Object *chassis);
extern void chassism_task(struct Struct_CHASSISM_Manage_Object *chassis);
extern const RC_ctrl_t *local_rc_ctrl;
#endif
