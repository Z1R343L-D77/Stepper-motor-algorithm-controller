/**
 * @file bsp_tpf.h
 * @author 19816
 * @brief 时间片任务框架头文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_002_G_BSP_TPF_H
#define INC_002_G_BSP_TPF_H

#ifdef __cplusplus
extern "C" {
#endif

/* 标准头文件 */
#include <stdint.h>

/* 宏定义 */
#define TP_MAX_TICK        (0xFFFFFFFFUL)

typedef enum
{
    TP_STATUS_OK = 0,
    TP_STATUS_ERROR = 1
} tp_status_e;

typedef enum
{
    TP_TASK_READY = 0,
    TP_TASK_BLOCKED = 1,
    TP_TASK_TERMINATED = 2
} tp_task_state_e;

typedef struct tp_task
{
    struct tp_task *last;
    struct tp_task *next;
    uint16_t id;
    uint32_t timeout_tick;
    tp_task_state_e state;
    uint32_t time_piece;
    void (*handler)(void);
} tp_task_t;

typedef tp_task_t tptask_t;

uint32_t TP_GetTick(void);
void TP_TickUpdate(uint32_t period);
void TP_TaskHandler(void);
tp_status_e TP_TaskInit(tp_task_t *handle, uint32_t time_piece, void (*task_handler)(void));
tp_status_e TP_TaskDelay(tp_task_t *handle, uint32_t ms);
tp_status_e TP_GlobalDelay(uint32_t ms);
tp_status_e TP_TaskTerminate(tp_task_t *handle);
tp_status_e TP_TaskRework(tp_task_t *handle);
void TP_ErrorHandler(tp_task_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* INC_002_G_BSP_TPF_H */

