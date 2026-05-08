/**
 * @file bsp_time.h
 * @author 19816
 * @brief 系统时基板级支持包头文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

/* 标准头文件 */
#include <stdint.h>

/* 工程头文件 */
#include "at32f403a_407.h"

/* 函数声明 */
void BSP_Timer_Init(void);
void BSP_TimerStepMain_Init(void);

/* 变量声明 */
extern volatile uint32_t systick_ms;

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIME_H */

