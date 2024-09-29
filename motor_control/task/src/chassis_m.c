/**
 * @file chassis_m.c
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "chassis_m.h"
struct Struct_CHASSISM_Manage_Object chassism_control = {0};

/**
 * @brief 发射处理初始化
 * @param 结构体地址
 */
void chassism_init(struct Struct_CHASSISM_Manage_Object *chassis)
{

  bsp_motor_init(&chassis->L1_motor, 0x201);
  bsp_motor_init(&chassis->L2_motor, 0x204);
  bsp_motor_init(&chassis->R2_motor, 0x203);
  bsp_motor_init(&chassis->R1_motor, 0x202);

  BSP_PID_Init(&(chassis->L1_motor.v_pid_object), 20, 0.03, 10, 5, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->L2_motor.v_pid_object), 20, 0.03, 10, 5, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->R1_motor.v_pid_object), 20, 0.03, 10, 5, 10000, 16000, 20);
  BSP_PID_Init(&(chassis->R2_motor.v_pid_object), 20, 0.03, 10, 5, 10000, 16000, 20);

chassis->chassis_motor[0].symbol[0]=1;
chassis->chassis_motor[0].symbol[1]=1;
chassis->chassis_motor[0].symbol[2]=1;
chassis->chassis_motor[0].symbol[3]=-1;
	
chassis->chassis_motor[1].symbol[0]=-1;
chassis->chassis_motor[1].symbol[1]=-1;
chassis->chassis_motor[1].symbol[2]=1;
chassis->chassis_motor[1].symbol[3]=1;
	
	
	
chassis->chassis_motor[2].symbol[0]=-1;
chassis->chassis_motor[2].symbol[1]=1;
chassis->chassis_motor[2].symbol[2]=-1;
chassis->chassis_motor[2].symbol[3]=1;

chassis->chassis_motor[3].symbol[0]=1;
chassis->chassis_motor[3].symbol[1]=-1;
chassis->chassis_motor[3].symbol[2]=-1;
chassis->chassis_motor[3].symbol[3]=-1;

  chassis->max_speed = 2000;
}

/**
 * @brief 确定底盘速度及角速度
 * @param 遥控器通道
 * @param 结构体地址
 */
void chassism_calculate(int16_t ch1, int16_t ch2, int16_t ch3, struct Struct_CHASSISM_Manage_Object *chassis)
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
    chassis->w = (float)(ch3 - 160) / 1000 * chassis->max_speed;
  else if (ch3 < -160)
    chassis->w = (float)(ch3 + 160) / 1000 * chassis->max_speed;
  else
    chassis->w = 0;
}

/**
 * @brief 底盘速度解算
 * @param 结构体地址
 */
void backwardm_calc(struct Struct_CHASSISM_Manage_Object *chassis)
{
  for (int i = 0; i < 4; i++)
  {
    chassis->chassis_motor[i].omega=(sqrt(2)/2*(chassis->vx+chassis->w*chassis->chassis_motor[i].symbol[0]*ry_chassism)+chassis->chassis_motor[i].symbol[1]*sqrt(2)/2*(chassis->vy+chassis->chassis_motor[i].symbol[2]*chassis->w*rx_chassism))/chassis->chassis_motor[i].symbol[3];
  }
}

/**
 * @brief 更改底盘每个电机的目标速度
 * @param 结构体地址
 */
void chassism_task(struct Struct_CHASSISM_Manage_Object *chassis)
{
  chassism_calculate(local_rc_ctrl->rc.ch[3], -local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[4], chassis);
  backwardm_calc(chassis);

  chassis->L1_motor.target_v = chassis->chassis_motor[1].omega;
  chassis->L2_motor.target_v = chassis->chassis_motor[2].omega;
  chassis->R2_motor.target_v = chassis->chassis_motor[3].omega;
  chassis->R1_motor.target_v = chassis->chassis_motor[0].omega;

  bsp_motor_state_change(&chassis->L1_motor, chassis->L1_motor.motor_cotrol_way, pid_control3, chassis->L1_motor.target_v);
  bsp_motor_state_change(&chassis->L2_motor, chassis->L2_motor.motor_cotrol_way, pid_control3, chassis->L2_motor.target_v);
  bsp_motor_state_change(&chassis->R1_motor, chassis->R1_motor.motor_cotrol_way, pid_control3, chassis->R1_motor.target_v);
  bsp_motor_state_change(&chassis->R2_motor, chassis->R2_motor.motor_cotrol_way, pid_control3, chassis->R2_motor.target_v);
  CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
}
