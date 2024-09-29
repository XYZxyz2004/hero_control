/**
 * @file shoot.h
 * @author xyz
 * @brief ��ɷ�������ĳ�ʼ�������岦���Ͳ�ͬ�ٷ���
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#ifndef CHASSIS_H
#define CHASSIS_H
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

#define rx_chassis 0.18
#define ry_chassis 0.18
#define s_chassis  0.125
#define pi 3.14159

typedef struct
{
	float rx, ry, s; // rx,ryΪ��ת�ᵽ�������ĵ�x��y���룬sΪȫ���ְ뾶
	float theta; // thetaΪ���� ��x��н�
	float omega; // omegaΪĿ��ת��
} motor_chassis;



struct Struct_CHASSIS_Manage_Object
{
	struct Struct_MOTOR_Manage_Object L1_motor ;
	struct Struct_MOTOR_Manage_Object L2_motor ;
	struct Struct_MOTOR_Manage_Object R1_motor ;
	struct Struct_MOTOR_Manage_Object R2_motor ;
	float vx,vy,w;          //����x,y��ת�٣���ת���ٶ�
  float max_speed;         //�����ƶ�����ٶ�
	motor_chassis chassis_motor[4];
};

extern void chassis_calculate(int16_t ch1,int16_t ch2,int16_t ch3,struct Struct_CHASSIS_Manage_Object *chassis);
extern void backward_calc(struct Struct_CHASSIS_Manage_Object *chassis) ;

extern struct Struct_CHASSIS_Manage_Object chassis_control;
extern void chassis_init(struct Struct_CHASSIS_Manage_Object *chassis);
extern void chassis_task(struct Struct_CHASSIS_Manage_Object *chassis);
extern const RC_ctrl_t *local_rc_ctrl;
#endif
