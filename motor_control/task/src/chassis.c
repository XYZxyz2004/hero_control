/**
 * @file chassis.c
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "chassis.h"
struct Struct_CHASSIS_Manage_Object chassis_control = {0};

/**
 * @brief 发射处理初始化
 * @param 结构体地址
 */
void chassis_init(struct Struct_CHASSIS_Manage_Object *chassis)
{
//电机初始化
  bsp_motor_init(&chassis->L1_motor, 0x201);
  bsp_motor_init(&chassis->L2_motor, 0x202);
  bsp_motor_init(&chassis->R2_motor, 0x203);
  bsp_motor_init(&chassis->R1_motor, 0x204);

	//底盘3508速度环 1.1
  BSP_PID_Init(&(chassis->L1_motor.v_pid_object), 5, 0.03, 0.3, 3, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->L2_motor.v_pid_object), 7, 0.03, 0.3, 3, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->R1_motor.v_pid_object), 5, 0.03, 0.3, 3, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->R2_motor.v_pid_object), 5, 0.03, 0.3, 3, 10000, 16000, 20);

  chassis->L1_motor.motor_cotrol_way = velocity_control;
  chassis->L1_motor.target_v = 0.0f;
  chassis->L2_motor.motor_cotrol_way = velocity_control;
  chassis->L2_motor.target_v = 0.0f;
  chassis->R1_motor.motor_cotrol_way = velocity_control;
  chassis->R1_motor.target_v = 0.0f;
  chassis->R2_motor.motor_cotrol_way = velocity_control;
  chassis->R2_motor.target_v = 0.0f;


//底盘运动解算初始化
  for (int i = 0; i < 4; i++)
  {
    chassis->chassis_motor[i].s = s_chassis;
  }
  chassis->chassis_motor[0].rx = rx_chassis;
  chassis->chassis_motor[0].ry = ry_chassis;
  chassis->chassis_motor[1].rx = -rx_chassis;
  chassis->chassis_motor[1].ry = ry_chassis;
  chassis->chassis_motor[2].rx = -rx_chassis;
  chassis->chassis_motor[2].ry = -ry_chassis;
  chassis->chassis_motor[3].rx = rx_chassis;
  chassis->chassis_motor[3].ry = -ry_chassis;

  chassis->chassis_motor[0].theta = pi / 4 * 3;
  chassis->chassis_motor[1].theta = -pi / 4 * 3;
  chassis->chassis_motor[2].theta = -pi / 4;
  chassis->chassis_motor[3].theta = pi / 4;

  chassis->max_speed = 1000;
}

/**
 * @brief 确定底盘速度及角速度
 * @param 遥控器通道
 * @param 结构体地址
 */
void chassis_calculate(int16_t ch1, int16_t ch2, int16_t ch3, struct Struct_CHASSIS_Manage_Object *chassis)
{
  if (ch1 > 160)
    chassis->vx = (float)(ch1 - 160) / 500 * chassis->max_speed;
  else if (ch1 < -160)
    chassis->vx = (float)(ch1 + 160) / 500 * chassis->max_speed;
  else
    chassis->vx = 0;

  if (ch2 > 160)
    chassis->vy = (float)(ch2 - 160) / 500 * chassis->max_speed;
  else if (ch2 < -160)
    chassis->vy = (float)(ch2 + 160) / 500 * chassis->max_speed;
  else
    chassis->vy = 0;

  if (ch3 > 160)
    chassis->w = (float)(ch3 - 160) / 500 * chassis->max_speed;
  else if (ch3 < -160)
    chassis->w = (float)(ch3 + 160) / 500 * chassis->max_speed;
  else
    chassis->w = 0;
}

/**
 * @brief 底盘速度解算
 * @param 结构体地址
 */
void backward_calc(struct Struct_CHASSIS_Manage_Object *chassis)
{
  for (int i = 0; i < 4; i++)
  {
    chassis->chassis_motor[i].omega = ((chassis->vx - chassis->w * chassis->chassis_motor[i].ry) * cos(chassis->chassis_motor[i].theta) + (chassis->vy + chassis->w * chassis->chassis_motor[i].rx) * sin(chassis->chassis_motor[i].theta)) / chassis->chassis_motor[i].s;
  }
}

/**
 * @brief 更改底盘每个电机的目标速度
 * @param 结构体地址
 */
void chassis_task(struct Struct_CHASSIS_Manage_Object *chassis)
{
		//计算限制速度
  chassis_calculate(local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3], local_rc_ctrl->rc.ch[4], chassis);
  backward_calc(chassis);
	

  chassis->L1_motor.target_v = chassis->chassis_motor[0].omega;
  chassis->L2_motor.target_v = chassis->chassis_motor[1].omega;
  chassis->R2_motor.target_v = chassis->chassis_motor[2].omega;
  chassis->R1_motor.target_v = chassis->chassis_motor[3].omega;

	//计算PID并发送
  bsp_motor_state_change(&chassis->L1_motor, chassis->L1_motor.motor_cotrol_way, pid_control3, chassis->L1_motor.target_v);
  bsp_motor_state_change(&chassis->L2_motor, chassis->L2_motor.motor_cotrol_way, pid_control3, chassis->L2_motor.target_v);
  bsp_motor_state_change(&chassis->R1_motor, chassis->R1_motor.motor_cotrol_way, pid_control3, chassis->R1_motor.target_v);
  bsp_motor_state_change(&chassis->R2_motor, chassis->R2_motor.motor_cotrol_way, pid_control3, chassis->R2_motor.target_v);
  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
}
