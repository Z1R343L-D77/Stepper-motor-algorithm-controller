/**
 * @file drv_gpio.c
 * @author 19816
 * @brief GPIO驱动（LED控制）源文件
 * @version 1.2
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include <stdio.h>
#include "drv_gpio.h"

/* LED1 控制实例，仅在本文件内使用 */
static gpio_control_t led1_handle;

/**
 * @brief GPIO控制状态机（5ms调用一次）
 * @param handle GPIO控制结构体指针
 * @retval 无
 */
static void GPIO_ControlWork(gpio_control_t *handle)
{
    uint16_t on_ticks;

    /* 入口先做空指针保护，防止异常访问 */
    if (handle == NULL)
    {
        return;
    }

    /* 处理复位请求，统一清零状态机上下文 */
    if (handle->reset_flag == 1U)
    {
        handle->reset_flag = 0U;
        handle->cycle_cnt = 0U;
        handle->completed_times = 0U;
        handle->finish_flag = 0U;
    }

    /* 达到目标次数后锁定完成状态并输出停止电平 */
    if (handle->completed_times >= handle->total_times)
    {
        handle->finish_flag = 1U;
        if (handle->active_level == GPIO_HIGH)
        {
            GPIO_OFF(handle);
        }
        else
        {
            GPIO_ON(handle);
        }
        return;
    }

    /* 推进周期计数，并按占空比计算亮灯阈值 */
    handle->cycle_cnt++;
    on_ticks = (uint16_t)(handle->period * handle->duty_cycle);

    /* 在亮灯区间输出有效电平 */
    if (handle->cycle_cnt <= on_ticks)
    {
        if (handle->active_level == GPIO_HIGH)
        {
            GPIO_ON(handle);
        }
        else
        {
            GPIO_OFF(handle);
        }
    }
    /* 在灭灯区间输出无效电平 */
    else if (handle->cycle_cnt < handle->period)
    {
        if (handle->active_level == GPIO_HIGH)
        {
            GPIO_OFF(handle);
        }
        else
        {
            GPIO_ON(handle);
        }
    }
    /* 单周期结束后回卷计数并累计完成次数 */
    else
    {
        handle->cycle_cnt = 0U;
        handle->completed_times++;
    }
}

/**
 * @brief GPIO初始化（LED控制结构体初始化）
 * @param 无
 * @retval 无
 */
void GPIO_Init(void)
{
    /* 配置 LED 输出端口与引脚 */
    led1_handle.port = GPIOB;
    led1_handle.pin = GPIO_PINS_3;

    /* 设置默认工作参数 */
    led1_handle.active_level = GPIO_LOW;
    led1_handle.period = 20U;
    led1_handle.duty_cycle = 0.5f;
    led1_handle.total_times = 5U;

    /* 置位复位标志，等待任务函数完成状态清零 */
    led1_handle.reset_flag = 1U;
}

/**
 * @brief LED1参数动态配置
 * @param period 闪烁周期（5ms单位）
 * @param duty_cycle 占空比（0~1）
 * @param total_times 闪烁总次数
 * @retval 无
 */
void GPIO_SetupLed1(uint32_t period, float duty_cycle, uint16_t total_times)
{
    /* 写入外部配置参数 */
    led1_handle.period = period;
    led1_handle.duty_cycle = duty_cycle;
    led1_handle.total_times = total_times;

    /* 置位复位标志，使新配置立即生效 */
    led1_handle.reset_flag = 1U;
}

/**
 * @brief LED任务处理函数（5ms调用）
 * @param 无
 * @retval 无
 */
void GPIO_TaskProc(void)
{
    /* 周期调用状态机推进输出 */
    GPIO_ControlWork(&led1_handle);
}

