/**
 * @file task_init.h
 * @author xyz
 * @brief 完成相关任务的板级支持包启动
 * @version 1.1
 * @date 2024-09-24
 * @copyright Transistor BUAA
 */
#ifndef TASK_INIT_H
#define TASK_INIT_H
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
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "remote_control.h"
extern void task_start_init();
extern const RC_ctrl_t *local_rc_ctrl;
#endif
