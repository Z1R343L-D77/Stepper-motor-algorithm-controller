/**
 * @file config.h
 * @author Ws
 * @brief 应用层公共配置头文件。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_002_G_BSP_SYSTEM_H
#define INC_002_G_BSP_SYSTEM_H

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "bsp_tpf.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_ringbuffer.h"
#include "drv_gpio.h"
#include "drv_button.h"
#include "f_cam.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化应用模块并运行主循环调度器。
 * @param 无。
 * @retval 无。
 */
#define loop  Loop

void Loop(void);

extern uint16_t uart_rx_index;
extern uint32_t uart_rx_ticks;
extern uint8_t uart_rx_buffer[128];
extern uint8_t uart_rx_dma_buffer[128];
extern uint32_t dma_buff[2][30];

#ifdef __cplusplus
}
#endif

#endif /* INC_002_G_BSP_SYSTEM_H */

