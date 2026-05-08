/**
 * @file drv_goseiko.c
 * @author 19816
 * @brief 电感式金属传感器驱动源文件
 * @version 1.2
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include <stdio.h>
#include "drv_goseiko.h"

/* 三路传感器控制实例，仅在本文件内使用 */
static goseiko_control_t goseiko_x1_handle;
static goseiko_control_t goseiko_x2_handle;
static goseiko_control_t goseiko_x3_handle;

/**
 * @brief 传感器状态机（带消抖）
 * @param handle 传感器控制结构体指针
 * @param goseiko_id 传感器 ID
 * @retval 无
 */
static void GOSEIKO_ControlWork(goseiko_control_t *handle, goseiko_id_e goseiko_id)
{
    goseiko_state_e raw_state;

    /* 状态机入口先做空指针保护 */
    if (handle == NULL)
    {
        return;
    }

    /* 读取当前原始状态 */
    raw_state = GOSEIKO_ReadRaw(goseiko_id);

    /* 状态变化时重置消抖计数并记录最新原始状态 */
    if (raw_state != handle->last_raw_state)
    {
        handle->debounce_cnt = 1U;
        handle->last_raw_state = raw_state;
    }
    else
    {
        /* 状态稳定时递增计数，直到达到阈值 */
        if (handle->debounce_cnt < handle->debounce_threshold)
        {
            handle->debounce_cnt++;
        }

        /* 达到阈值后更新稳定状态，过滤抖动 */
        if (handle->debounce_cnt >= handle->debounce_threshold)
        {
            handle->current_state = raw_state;
        }
    }
}

/**
 * @brief 传感器初始化
 * @param 无
 * @retval 无
 */
void GOSEIKO_Init(void)
{
    /* 初始化 X1 传感器参数与状态 */
    goseiko_x1_handle.port = GPIOA;
    goseiko_x1_handle.pin = GPIO_PINS_6;
    goseiko_x1_handle.current_state = GOSEIKO_NO_METAL;
    goseiko_x1_handle.last_raw_state = GOSEIKO_NO_METAL;
    goseiko_x1_handle.debounce_cnt = 0U;
    goseiko_x1_handle.debounce_threshold = DEBOUNCE_THRESHOLD;

    /* 初始化 X2 传感器参数与状态 */
    goseiko_x2_handle.port = GPIOC;
    goseiko_x2_handle.pin = GPIO_PINS_9;
    goseiko_x2_handle.current_state = GOSEIKO_NO_METAL;
    goseiko_x2_handle.last_raw_state = GOSEIKO_NO_METAL;
    goseiko_x2_handle.debounce_cnt = 0U;
    goseiko_x2_handle.debounce_threshold = DEBOUNCE_THRESHOLD;

    /* 初始化 X3 传感器参数与状态 */
    goseiko_x3_handle.port = GPIOA;
    goseiko_x3_handle.pin = GPIO_PINS_11;
    goseiko_x3_handle.current_state = GOSEIKO_NO_METAL;
    goseiko_x3_handle.last_raw_state = GOSEIKO_NO_METAL;
    goseiko_x3_handle.debounce_cnt = 0U;
    goseiko_x3_handle.debounce_threshold = DEBOUNCE_THRESHOLD;
}

/**
 * @brief 读取传感器原始状态（无消抖）
 * @param goseiko_id 传感器 ID（GOSEIKO_X1/GOSEIKO_X2/GOSEIKO_X3）
 * @retval goseiko_state_e GOSEIKO_NO_METAL 或 GOSEIKO_DETECTED
 */
goseiko_state_e GOSEIKO_ReadRaw(goseiko_id_e goseiko_id)
{
    goseiko_control_t *handle = NULL;

    /* 根据 ID 选择目标传感器控制对象 */
    switch (goseiko_id)
    {
        case GOSEIKO_X1:
        {
            handle = &goseiko_x1_handle;
            break;
        }
        case GOSEIKO_X2:
        {
            handle = &goseiko_x2_handle;
            break;
        }
        case GOSEIKO_X3:
        {
            handle = &goseiko_x3_handle;
            break;
        }
        default:
        {
            return GOSEIKO_NO_METAL;
        }
    }

    /* 防御性空指针检查 */
    if (handle == NULL)
    {
        return GOSEIKO_NO_METAL;
    }

    /* PNP 型传感器低电平表示检测到金属 */
    if (GOSEIKO_READ(handle) == RESET)
    {
        return GOSEIKO_DETECTED;
    }

    return GOSEIKO_NO_METAL;
}

/**
 * @brief 获取传感器稳定状态
 * @param goseiko_id 传感器 ID（GOSEIKO_X1/GOSEIKO_X2/GOSEIKO_X3）
 * @retval uint8_t 0 表示无金属，1 表示检测到金属
 */
uint8_t GOSEIKO_GetState(goseiko_id_e goseiko_id)
{
    /* 按通道返回当前稳定状态 */
    switch (goseiko_id)
    {
        case GOSEIKO_X1:
        {
            return (uint8_t)goseiko_x1_handle.current_state;
        }
        case GOSEIKO_X2:
        {
            return (uint8_t)goseiko_x2_handle.current_state;
        }
        case GOSEIKO_X3:
        {
            return (uint8_t)goseiko_x3_handle.current_state;
        }
        default:
        {
            return (uint8_t)GOSEIKO_NO_METAL;
        }
    }
}

/**
 * @brief 传感器任务处理函数（2ms调用）
 * @param 无
 * @retval 无
 */
void GOSEIKO_TaskScanProc(void)
{
    /* 推进 X1 通道状态机 */
    GOSEIKO_ControlWork(&goseiko_x1_handle, GOSEIKO_X1);

    /* 推进 X2 通道状态机 */
    GOSEIKO_ControlWork(&goseiko_x2_handle, GOSEIKO_X2);

    /* 推进 X3 通道状态机 */
    GOSEIKO_ControlWork(&goseiko_x3_handle, GOSEIKO_X3);
}

