/**
 * @file bsp_tpf.h
 * @author Ws
 * @brief 时间片调度框架接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef TPF_H
#define TPF_H

#include <stdint.h>

#define TP_MAX_TICK       (0xFFFFFFFFUL)
#define TP_GetTick        TpGetTick
#define TP_TickUpdate     TpTickUpdate
#define tp_tick_update    TpTickUpdate
#define TP_TaskHandler    TpTaskHandler
#define TP_TaskInit       TpTaskInit
#define TP_TaskDelay      TpTaskDelay
#define TP_GlobalDelay    TpGlobalDelay
#define TP_TaskTerminate  TpTaskTerminate
#define TP_TaskRework     TpTaskRework

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TP_STATUS_OK = 0,
    TP_STATUS_ERROR
} tp_status_e;

typedef enum
{
    TP_TASK_READY = 0,
    TP_TASK_BLOCKED,
    TP_TASK_TERMINATED
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


/**
 * @brief 获取当前时间片框架 tick 计数。
 * @param 无。
 * @retval 当前时间片 tick 值。
 */
uint32_t TpGetTick(void);

/**
 * @brief 累加时间片框架 tick 计数。
 * @param period 调度周期提供的 tick 增量。
 * @retval 无。
 */
void TpTickUpdate(uint32_t period);

/**
 * @brief 执行一次时间片任务调度。
 * @param 无。
 * @retval 无。
 */
void TpTaskHandler(void);

/**
 * @brief 初始化时间片任务并加入调度链表。
 * @param handle 任务控制块指针。
 * @param time_piece 任务执行间隔，单位为框架 tick。
 * @param task_handler 任务就绪时调用的处理函数。
 * @retval TP_STATUS_OK 表示初始化成功。
 * @retval TP_STATUS_ERROR 表示参数无效。
 */
tp_status_e TpTaskInit(tp_task_t *handle, uint32_t time_piece, void (*task_handler)(void));

/**
 * @brief 将指定任务阻塞一段时间。
 * @param handle 任务控制块指针。
 * @param ms 阻塞时长，单位为框架 tick。
 * @retval TP_STATUS_OK 表示任务阻塞成功。
 * @retval TP_STATUS_ERROR 表示参数无效或任务已经终止。
 */
tp_status_e TpTaskDelay(tp_task_t *handle, uint32_t ms);

/**
 * @brief 将所有未终止任务阻塞一段时间。
 * @param ms 阻塞时长，单位为框架 tick。
 * @retval TP_STATUS_OK 表示处理完成。
 */
tp_status_e TpGlobalDelay(uint32_t ms);

/**
 * @brief 将任务标记为终止状态。
 * @param handle 任务控制块指针。
 * @retval TP_STATUS_OK 表示状态更新成功。
 * @retval TP_STATUS_ERROR 表示参数无效。
 */
tp_status_e TpTaskTerminate(tp_task_t *handle);

/**
 * @brief 将任务恢复为就绪状态并刷新超时 tick。
 * @param handle 任务控制块指针。
 * @retval TP_STATUS_OK 表示状态更新成功。
 * @retval TP_STATUS_ERROR 表示参数无效。
 */
tp_status_e TpTaskRework(tp_task_t *handle);

/**
 * @brief 处理任务调度中的异常状态。
 * @param handle 触发错误的任务控制块指针。
 * @retval 无。
 */
void TpErrorHandler(tp_task_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* TPF_H */

