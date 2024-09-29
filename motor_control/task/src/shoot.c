/**
 * @file shoot.c
 * @author xyz
 * @brief 完成发射任务的初始化及具体拨弹和不同速发射
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "shoot.h"
 struct Struct_SHOOT_Manage_Object shoot_control={0};

 
 /**
 * @brief 发射处理初始化
 * @param 结构体地址
 */
void shoot_init(struct Struct_SHOOT_Manage_Object *shoot)
{

	bsp_motor_init(&shoot->fric_wheel_l_motor,0x205);
	bsp_motor_init(&shoot->fric_wheel_r_motor,0x206);
	bsp_motor_init(&shoot->pull_bullets_motor,0x207);
//拨弹2006速度环 1.2
BSP_PID_Init(&(shoot->pull_bullets_motor.v_pid_object),5,0.01,10,0,10000,16000,20);
//拨弹2006位置环	 1.2
BSP_PID_Init(&shoot->pull_bullets_motor.l_pid_object,0.5,0,6,0,1000,16000,20);

//摩擦轮发射3508速度环 1.1
BSP_PID_Init(&shoot->fric_wheel_l_motor.v_pid_object,10,0.01,35,0,10000,16000,20);
BSP_PID_Init(&shoot->fric_wheel_r_motor.v_pid_object,10,0.01,35,0,10000,16000,20);

	
shoot->pull_bullets_motor.motor_cotrol_way=location_control;
shoot->pull_bullets_motor.target_location=0.0f;	
	
shoot->fric_wheel_l_motor.motor_cotrol_way=velocity_control;
shoot->fric_wheel_l_motor.target_v=0.0f;
	
shoot->fric_wheel_r_motor.motor_cotrol_way=velocity_control;
shoot->fric_wheel_r_motor.target_v=0.0f;
	
shoot->fric_flag=0;
shoot->pull_bullets_state=3;

}

 /**
 * @brief 根据大疆DBUS控制器s的变化控制摩擦轮的速度和每次拨一个弹
 * @param 结构体地址
 */
void shoot_task(struct Struct_SHOOT_Manage_Object *shoot)
{
	if((uint8_t)((*local_rc_ctrl).rc.s[0])==1)
	{
		shoot->fric_wheel_l_motor.target_v=-1000.0f;
		shoot->fric_wheel_r_motor.target_v=1000.0f;
		shoot->fric_flag=1;
	}
		if((uint8_t)((*local_rc_ctrl).rc.s[0])==2)
	{
		shoot->fric_wheel_l_motor.target_v=-5000.0f;
		shoot->fric_wheel_r_motor.target_v=5000.0f;
		shoot->fric_flag=1;
	}
		if((uint8_t)((*local_rc_ctrl).rc.s[0])==3)
	{
	shoot->fric_wheel_l_motor.target_v=0.0f;
		shoot->fric_wheel_r_motor.target_v=0.0f;
		shoot->fric_flag=0;
	}
//确保摩擦轮有速度时，改变拨动开关位置即完成一次拨弹	
if(shoot->fric_flag==1&&(uint8_t)((*local_rc_ctrl).rc.s[1])!=(*shoot).pull_bullets_state)
{	
	//计算限制速度
	shoot->pull_bullets_motor.target_location+=36*1.0/8;
	shoot->pull_bullets_state=(uint8_t)((*local_rc_ctrl).rc.s[1]);
}
//计算PID
bsp_motor_state_change(&shoot->pull_bullets_motor,shoot->pull_bullets_motor.motor_cotrol_way,pid_control2,shoot->pull_bullets_motor.target_location);
bsp_motor_state_change(&shoot->fric_wheel_l_motor,shoot->fric_wheel_l_motor.motor_cotrol_way,pid_control2,shoot->fric_wheel_l_motor.target_v);
bsp_motor_state_change(&shoot->fric_wheel_r_motor,shoot->fric_wheel_r_motor.motor_cotrol_way,pid_control2,shoot->fric_wheel_r_motor.target_v);
	CAN_Send_Data(&hcan1,0x1ff,CAN1_0x1ff_Tx_Data,8);
}
