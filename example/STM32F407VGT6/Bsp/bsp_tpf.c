/**
 * @file bsp_tpf.c
 * @author Ws
 * @brief 时间片调度框架实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#include "bsp_tpf.h"
#include <stdio.h>
static volatile uint32_t tp_tick;
static uint16_t task_count = 0U;
static tp_task_t *head;
static tp_task_t *tail;

/**
 * @brief 默认弱定义错误处理函数，用于处理异常时间片任务。
 * @param handle 触发错误的任务控制块指针。
 * @retval 无。
 */
__attribute__((weak)) void TpErrorHandler(tp_task_t *handle)
{
    (void)handle;

    while (1)
    {
        /* 进入异常处理后停留在此处，便于调试定位。 */
    }
}

/**
 * @brief 获取当前时间片框架 tick 计数。
 * @param 无。
 * @retval 当前时间片 tick 值。
 */
uint32_t TpGetTick(void)
{
    return tp_tick;
}

/**
 * @brief 累加时间片框架 tick 计数。
 * @param period 硬件定时器或调度周期提供的 tick 增量。
 * @retval 无。
 */
void TpTickUpdate(uint32_t period)
{
    tp_tick += period;
}

/**
 * @brief 初始化时间片任务并加入调度链表。
 * @param handle 任务控制块指针。
 * @param time_piece 任务执行间隔，单位为框架 tick。
 * @param task_handler 任务就绪时调用的处理函数。
 * @retval TP_STATUS_OK 任务初始化成功。
 * @retval TP_STATUS_ERROR 参数无效。
 */
tp_status_e TpTaskInit(tp_task_t *handle, uint32_t time_piece, void (*task_handler)(void))
{
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    if (head == NULL)
    {
        handle->last = NULL;
        handle->next = NULL;
        head = handle;
        tail = handle;
    }
    else
    {
        tail->next = handle;
        handle->last = tail;
        handle->next = NULL;
        tail = handle;
    }

    handle->id = task_count++;
    handle->state = TP_TASK_READY;
    handle->handler = task_handler;
    handle->time_piece = time_piece;
    handle->timeout_tick = handle->time_piece;

    return TP_STATUS_OK;
}

/**
 * @brief 执行一次时间片任务调度。
 * @param 无。
 * @retval 无。
 */
void TpTaskHandler(void)
{
    tp_task_t *handle;

    for (handle = head; handle != NULL; handle = handle->next)
    {
        switch (handle->state)
        {
            case TP_TASK_READY:
                if ((tp_tick - handle->timeout_tick) <= (TP_MAX_TICK / 2U))
                {
                    handle->timeout_tick = tp_tick + handle->time_piece;
                    if (handle->handler != NULL)
                    {
                        handle->handler();
                    }
                    else
                    {
                        TpErrorHandler(handle);
                    }
                }
                break;

            case TP_TASK_BLOCKED:
                if ((tp_tick - handle->timeout_tick) <= (TP_MAX_TICK / 2U))
                {
                    handle->timeout_tick = tp_tick + handle->time_piece;
                    handle->state = TP_TASK_READY;
                }
                break;

            case TP_TASK_TERMINATED:
                break;

            default:
                TpErrorHandler(handle);
                break;
        }
    }
}

/**
 * @brief 将指定任务阻塞一段时间。
 * @param handle 任务控制块指针。
 * @param ms 阻塞时长，单位为框架 tick。
 * @retval TP_STATUS_OK 任务阻塞成功。
 * @retval TP_STATUS_ERROR 参数无效或任务已经终止。
 */
tp_status_e TpTaskDelay(tp_task_t *handle, uint32_t ms)
{
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    if (handle->state == TP_TASK_TERMINATED)
    {
        return TP_STATUS_ERROR;
    }

    handle->state = TP_TASK_BLOCKED;
    handle->timeout_tick = tp_tick + ms;
    return TP_STATUS_OK;
}

/**
 * @brief 将所有未终止任务阻塞一段时间。
 * @param ms 阻塞时长，单位为框架 tick。
 * @retval TP_STATUS_OK 所有可处理任务均已更新。
 */
tp_status_e TpGlobalDelay(uint32_t ms)
{
    tp_task_t *handle;

    for (handle = head; handle != NULL; handle = handle->next)
    {
        if (handle->state == TP_TASK_TERMINATED)
        {
            continue;
        }

        handle->state = TP_TASK_BLOCKED;
        handle->timeout_tick = tp_tick + ms;
    }

    return TP_STATUS_OK;
}

/**
 * @brief 将任务标记为终止状态。
 * @param handle 任务控制块指针。
 * @retval TP_STATUS_OK 任务状态更新成功。
 * @retval TP_STATUS_ERROR 参数无效。
 */
tp_status_e TpTaskTerminate(tp_task_t *handle)
{
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    handle->state = TP_TASK_TERMINATED;
    handle->timeout_tick = 0U;
    return TP_STATUS_OK;
}

/**
 * @brief 将任务恢复为就绪状态并刷新超时 tick。
 * @param handle 任务控制块指针。
 * @retval TP_STATUS_OK 任务状态更新成功。
 * @retval TP_STATUS_ERROR 参数无效。
 */
tp_status_e TpTaskRework(tp_task_t *handle)
{
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    handle->state = TP_TASK_READY;
    handle->timeout_tick = tp_tick + handle->time_piece;
    return TP_STATUS_OK;
}

