/**
 * @file tripod_head.c
 * @author xyz
 * @brief 完成云台俯仰与偏航
 * @version 1.1
 * @date 2024-09-29
 * @copyright Transistor BUAA
 */

#include "tripod_head.h"
#include "math.h"
struct Struct_TRIPOD_Manage_Object tripod_head_control;

void tripod_init(struct Struct_TRIPOD_Manage_Object *tripod)
{
	
bsp_motor_init(&tripod->pitch_motor,0x209);
bsp_motor_init(&tripod->yaw_motor,0x20A);

	
tripod->pitch_start_location=0.0f;
tripod->yaw_start_location=0.0f;
	
BSP_PID_Init(&tripod->pitch_motor.v_pid_object,10,0,10,0,10000,16000,20);
BSP_PID_Init(&tripod->pitch_motor.l_pid_object,5,0,3,0,1000,16000,20);

BSP_PID_Init(&tripod->yaw_motor.v_pid_object,0,0,0,0,10000,16000,20);
BSP_PID_Init(&tripod->yaw_motor.l_pid_object,0,0,0,0,10000,16000,20);

	
tripod->pitch_motor.motor_cotrol_way=location_control;
tripod->pitch_motor.target_location=tripod->pitch_start_location;	

tripod->yaw_motor.motor_cotrol_way=velocity_control;
tripod->yaw_motor.target_location=tripod->yaw_start_location;		

}

void tripod_task(struct Struct_TRIPOD_Manage_Object *tripod)
{

CAN_Send_Data(&hcan1,0x2ff,CAN1_0x2ff_Tx_Data,8);
}