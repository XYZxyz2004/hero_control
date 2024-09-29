/**
 * @file bsp_can.h
 * @author xyz
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 1.1
 * @date 2024-09-13
 * @copyright Transistor BUAA
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧   决定IDE位
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧  决定RTR位
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)






/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN接收的信息结构体
 *
 */
struct Struct_CAN_Rx_Buffer
{//header包括了  标准帧/拓展帧   数据帧/遥控帧   数据长度 等因素
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
};



/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(struct Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信接收处理结构体
 *
 */
struct Struct_CAN_Manage_Object
{
    CAN_HandleTypeDef *CAN_Handler;
   struct Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;

};

/* Exported variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;
// extern CAN_HandleTypeDef hcan2;

extern struct Struct_CAN_Manage_Object CAN1_Manage_Object;
extern struct Struct_CAN_Manage_Object CAN2_Manage_Object;

//用于向电调发送控制指令控制电调的电流输出
extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t  CAN1_0x2ff_Tx_Data[];

extern void can_fifo_callback( struct Struct_CAN_Rx_Buffer *Rx_Buffer);

extern void buffer_to_motor_state(uint8_t id,struct Struct_CAN_Rx_Buffer *Rx_Buffer);

extern void TIM_CAN_PeriodElapsedCallback();
/* Exported function declarations ---------------------------------------------*/

extern void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);

extern void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

extern void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
