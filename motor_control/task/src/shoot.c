/**
 * @file shoot.c
 * @author xyz
 * @brief ��ɷ�������ĳ�ʼ�������岦���Ͳ�ͬ�ٷ���
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */

#include "shoot.h"
 struct Struct_SHOOT_Manage_Object shoot_control={0};

 
 /**
 * @brief ���䴦���ʼ��
 * @param �ṹ���ַ
 */
void shoot_init(struct Struct_SHOOT_Manage_Object *shoot)
{

	bsp_motor_init(&shoot->fric_wheel_l_motor,0x205);
	bsp_motor_init(&shoot->fric_wheel_r_motor,0x206);
	bsp_motor_init(&shoot->pull_bullets_motor,0x207);
//����2006�ٶȻ� 1.2
BSP_PID_Init(&(shoot->pull_bullets_motor.v_pid_object),5,0.01,10,0,10000,16000,20);
//����2006λ�û�	 1.2
BSP_PID_Init(&shoot->pull_bullets_motor.l_pid_object,0.5,0,6,0,1000,16000,20);

//Ħ���ַ���3508�ٶȻ� 1.1
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
 * @brief ���ݴ�DBUS������s�ı仯����Ħ���ֵ��ٶȺ�ÿ�β�һ����
 * @param �ṹ���ַ
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
//ȷ��Ħ�������ٶ�ʱ���ı䲦������λ�ü����һ�β���	
if(shoot->fric_flag==1&&(uint8_t)((*local_rc_ctrl).rc.s[1])!=(*shoot).pull_bullets_state)
{	
	//���������ٶ�
	shoot->pull_bullets_motor.target_location+=36*1.0/8;
	shoot->pull_bullets_state=(uint8_t)((*local_rc_ctrl).rc.s[1]);
}
//����PID
bsp_motor_state_change(&shoot->pull_bullets_motor,shoot->pull_bullets_motor.motor_cotrol_way,pid_control2,shoot->pull_bullets_motor.target_location);
bsp_motor_state_change(&shoot->fric_wheel_l_motor,shoot->fric_wheel_l_motor.motor_cotrol_way,pid_control2,shoot->fric_wheel_l_motor.target_v);
bsp_motor_state_change(&shoot->fric_wheel_r_motor,shoot->fric_wheel_r_motor.motor_cotrol_way,pid_control2,shoot->fric_wheel_r_motor.target_v);
	CAN_Send_Data(&hcan1,0x1ff,CAN1_0x1ff_Tx_Data,8);
}
