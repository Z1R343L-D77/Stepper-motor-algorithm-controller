/**
 * @file bsp_tpf.c
 * @author 19816
 * @brief 时间片任务框架源文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include <stdio.h>
#include "bsp_tpf.h"
#include <stddef.h>

static volatile uint32_t tp_tick = 0U;
static uint16_t task_count = 0U;
static tp_task_t *head = NULL;
static tp_task_t *tail = NULL;

/**
 * @brief 默认错误处理函数（可被外部重载）
 * @param handle 任务句柄
 * @retval 无
 */
__attribute__((weak)) void TP_ErrorHandler(tp_task_t *handle)
{
    (void)handle;
    while (1)
    {
    }
}

/**
 * @brief 获取当前系统节拍计数
 * @param 无
 * @retval uint32_t 当前节拍值
 */
uint32_t TP_GetTick(void)
{
    return tp_tick;
}

/**
 * @brief 更新系统节拍计数
 * @param period 本次增加的节拍数
 * @retval 无
 */
void TP_TickUpdate(uint32_t period)
{
    tp_tick += period;
}

/**
 * @brief 初始化任务并挂入任务链表
 * @param handle 任务句柄
 * @param time_piece 任务时间片
 * @param task_handler 任务处理函数
 * @retval tp_status_e 初始化结果
 */
tp_status_e TP_TaskInit(tp_task_t *handle, uint32_t time_piece, void (*task_handler)(void))
{
    /* 校验输入参数，防止空指针和非法时间片 */
    if ((handle == NULL) || (task_handler == NULL) || (time_piece == 0U))
    {
        return TP_STATUS_ERROR;
    }

    /* 首次挂载时初始化头尾指针，否则追加到链表尾部 */
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

    /* 初始化任务运行参数 */
    handle->id = task_count++;
    handle->state = TP_TASK_READY;
    handle->handler = task_handler;
    handle->time_piece = time_piece;
    handle->timeout_tick = handle->time_piece;

    return TP_STATUS_OK;
}

/**
 * @brief 任务调度处理函数
 * @param 无
 * @retval 无
 */
void TP_TaskHandler(void)
{
    tp_task_t *handle;

    /* 遍历任务链表，按状态机推进每个任务 */
    for (handle = head; handle != NULL; handle = handle->next)
    {
        switch (handle->state)
        {
            case TP_TASK_READY:
            {
                if ((tp_tick - handle->timeout_tick) <= (TP_MAX_TICK / 2U))
                {
                    handle->timeout_tick = tp_tick + handle->time_piece;
                    handle->handler();
                }
                break;
            }

            case TP_TASK_BLOCKED:
            {
                if ((tp_tick - handle->timeout_tick) <= (TP_MAX_TICK / 2U))
                {
                    handle->timeout_tick = tp_tick + handle->time_piece;
                    handle->state = TP_TASK_READY;
                }
                break;
            }

            case TP_TASK_TERMINATED:
            {
                break;
            }

            default:
            {
                TP_ErrorHandler(handle);
                break;
            }
        }
    }
}

/**
 * @brief 任务延时
 * @param handle 任务句柄
 * @param ms 延时时间（ms）
 * @retval tp_status_e 处理结果
 */
tp_status_e TP_TaskDelay(tp_task_t *handle, uint32_t ms)
{
    /* 非法任务或已终止任务直接返回错误 */
    if ((handle == NULL) || (handle->state == TP_TASK_TERMINATED))
    {
        return TP_STATUS_ERROR;
    }

    /* 切换为阻塞态并设置超时时间点 */
    handle->state = TP_TASK_BLOCKED;
    handle->timeout_tick = tp_tick + ms;
    return TP_STATUS_OK;
}

/**
 * @brief 所有任务统一延时
 * @param ms 延时时间（ms）
 * @retval tp_status_e 处理结果
 */
tp_status_e TP_GlobalDelay(uint32_t ms)
{
    tp_task_t *handle;

    /* 对所有未终止任务应用统一阻塞延时 */
    for (handle = head; handle != NULL; handle = handle->next)
    {
        if (handle->state != TP_TASK_TERMINATED)
        {
            handle->state = TP_TASK_BLOCKED;
            handle->timeout_tick = tp_tick + ms;
        }
    }

    return TP_STATUS_OK;
}

/**
 * @brief 终止任务
 * @param handle 任务句柄
 * @retval tp_status_e 处理结果
 */
tp_status_e TP_TaskTerminate(tp_task_t *handle)
{
    /* 空句柄直接返回错误 */
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    /* 标记终止状态并清空超时计数 */
    handle->state = TP_TASK_TERMINATED;
    handle->timeout_tick = 0U;
    return TP_STATUS_OK;
}

/**
 * @brief 重置任务为就绪状态
 * @param handle 任务句柄
 * @retval tp_status_e 处理结果
 */
tp_status_e TP_TaskRework(tp_task_t *handle)
{
    /* 空句柄直接返回错误 */
    if (handle == NULL)
    {
        return TP_STATUS_ERROR;
    }

    /* 重新置为就绪并更新下次触发时刻 */
    handle->state = TP_TASK_READY;
    handle->timeout_tick = tp_tick + handle->time_piece;
    return TP_STATUS_OK;
}

