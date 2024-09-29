/**
 * @file task_init.c
 * @author xyz
 * @brief 完成相关任务的板级支持包启动
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#include "task_init.h"
#include "shoot.h"
#include "tripod_head.h"
#include "chassis_m.h"
void task_start_init()
 { 
	 //串口发送绘图启动
  //  usart1_tx_dma_init();


	 
	 
	shoot_init(&shoot_control);
	tripod_init(&tripod_head_control);
	chassism_init(&chassism_control);
	
	CAN_Init(&hcan1, can_fifo_callback);
	CAN_Init(&hcan2, can_fifo_callback);

	
	remote_control_init();
local_rc_ctrl = get_remote_control_point();

 }