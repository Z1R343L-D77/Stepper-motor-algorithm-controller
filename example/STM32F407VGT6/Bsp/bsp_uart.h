/**
 * @file bsp_uart.h
 * @author Ws
 * @brief UART 板级通信接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef UART_APP_H
#define UART_APP_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART 接收缓冲与命令处理周期任务。
 * @param 无。
 * @retval 无。
 */
void UartProc(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_APP_H */

