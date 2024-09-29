/**
 * @file bsp_rc.h
 * @author xyz
 * @brief 大疆遥控器对应的USART3串口配置过程
 * @version 1.1
 * @date 2024-09-21
 * @copyright Transistor BUAA
 */

#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"

extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
#endif
