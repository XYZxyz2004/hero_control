/**
 * @file bsp_can.c
 * @author xyz
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 1.1
 * @date 2024-09-13
 * @copyright Transistor BUAA
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

#include "usart.h"
#include "bsp_motor.h"
#include "shoot.h"
#include "chassis_m.h"
#include "tripod_head.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CAN通信发送缓冲区
//控制两个摩擦轮、1个拨盘2006电机
uint8_t CAN1_0x1ff_Tx_Data[8]= {0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//控制底盘四个3508电机
uint8_t  CAN1_0x200_Tx_Data[8] = {0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//控制云台俯仰 偏航6020电机 
uint8_t  CAN1_0x2ff_Tx_Data[8] = {0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
			
		//对ID处于0x200-0x20f的ID进行过滤
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x201, 0xfff);
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x202, 0xfff);
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(2) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x203, 0xfff);	
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(3) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x204, 0xfff);	
			
			
			
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(4) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x205, 0xfff);	
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(5) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x206, 0xfff);	
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(6) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x207, 0xfff);	
			
			
			
			CAN_Filter_Mask_Config(hcan, CAN_FILTER(7) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x209, 0xfff);	
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
			//不进行任何过滤
	//	CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x000, 0x000);
		//CAN_Filter_Mask_Config(hcan, CAN_FILTER(16) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x000, 0x000);	

    
    }
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位
 */
 
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //检测传参是否正确
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        //拓展帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 >> 16;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        //标准帧
        //掩码后ID的高16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        //掩码后ID的低16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID掩码值高16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID掩码值低16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    //滤波器序号, 0-27, 共28个滤波器, 前14个在CAN1, 后14个在CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //滤波器绑定FIFO0或FIFO1
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //使能滤波器
    can_filter_init_structure.FilterActivation = ENABLE;
    //滤波器模式，设置ID掩码模式
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32位滤波
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}


/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback()
{
    // CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);
    //CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
}

/**
 * @brief 发送数据帧
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确

    tx_header.StdId = ID;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = Length;

    HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox);
}


/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// 此函数是CAN接收FIFO0消息挂起回调函数，用于处理CAN1和CAN2的接收消息。
{	
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
			
      HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
		 CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
			
    }
    else if (hcan->Instance == CAN2)
    {		  
       HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
		 CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
		
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //选择回调函数
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
			 CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
       CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
}

/**
 * @brief 待定义将接收报文与对应电机结构体绑定起来，存储相关数据为下一步PID计算做准备
 * @param 电调id
 * @param CAN接收的信息结构体
 */
void buffer_to_motor_state(uint8_t id,struct Struct_CAN_Rx_Buffer *Rx_Buffer)
{
	switch(id)
	{
		case(1):
		{
			  chassism_control.L1_motor.encoder =(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        chassism_control.L1_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        chassism_control.L1_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
       chassism_control.L1_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
		case(2):
		{
			 chassism_control.R1_motor.encoder =(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        chassism_control.R1_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        chassism_control.R1_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
       chassism_control.R1_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
			case(3):
		{
			 chassism_control.R2_motor.encoder =(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        chassism_control.R2_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        chassism_control.R2_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
       chassism_control.R2_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
			case(4):
		{
			  chassism_control.L2_motor.encoder =(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        chassism_control.L2_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        chassism_control.L2_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
       chassism_control.L2_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
			case(5):
		{
			  shoot_control.fric_wheel_l_motor.encoder=(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        shoot_control.fric_wheel_l_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        shoot_control.fric_wheel_l_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
        shoot_control.fric_wheel_l_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
			case(6):
		{
			  shoot_control.fric_wheel_r_motor.encoder=(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        shoot_control.fric_wheel_r_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        shoot_control.fric_wheel_r_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
        shoot_control.fric_wheel_r_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
			case(7):
		{
			  shoot_control.pull_bullets_motor.encoder=(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        shoot_control.pull_bullets_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        shoot_control.pull_bullets_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
        shoot_control.pull_bullets_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
		
				case(8):
		{
			
		}
		break;
			case(9):
		{
			  tripod_head_control.pitch_motor.encoder=(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
       tripod_head_control.pitch_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
        tripod_head_control.pitch_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
        tripod_head_control.pitch_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
		case(10):
		{
			  tripod_head_control.yaw_motor.encoder=(uint16_t)Rx_Buffer->Data[0]<<8|Rx_Buffer->Data[1];
        tripod_head_control.yaw_motor.omega=(int16_t)Rx_Buffer->Data[2]<<8|Rx_Buffer->Data[3];
         tripod_head_control.yaw_motor.torque=(int16_t)Rx_Buffer->Data[4]<<8|Rx_Buffer->Data[5];
        tripod_head_control.yaw_motor.temperaure=Rx_Buffer->Data[6];
		}
		break;
		case(11):
		{
			  
		}
			break;
	}
}

/**
 * @brief 对fifo转变而来的buffer中的数据进行处理的自定义回调函数
 * @param CAN接收的信息结构体
 */
void can_fifo_callback( struct Struct_CAN_Rx_Buffer *Rx_Buffer)
{
		switch (Rx_Buffer->Header.StdId)
    {
        case (0x201):
        {
						buffer_to_motor_state(1,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据
			
        }
        break;
        case (0x202):
        {
							buffer_to_motor_state(2,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据

        }
        break;
        case (0x203):
        {
							buffer_to_motor_state(3,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据
	
				}
				
        break;

        case (0x204):
        {		buffer_to_motor_state(4,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据
	
        }
        break;
        case (0x205):
        {  
					buffer_to_motor_state(5,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据
			
        }
        break;
        case (0x206):
        {
        	buffer_to_motor_state(6,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据

        }
        break;
        case (0x207):
        {
        buffer_to_motor_state(7,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据

        }
        break;
        case (0x208):
        {
        
        }
        break;
				 case (0x209):
        {
        buffer_to_motor_state(9,Rx_Buffer);//buffer得到的数据进行处理得到电机基础数据
	
        }
        break;
				 case (0x20A):
        {
         buffer_to_motor_state(10,Rx_Buffer);
        }
        break;
				 case (0x20B):
        {
        
        }
        break;
    }
}
