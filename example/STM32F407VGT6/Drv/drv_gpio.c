/**
 * @file drv_gpio.c
 * @author Ws
 * @brief GPIO 控制驱动实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "drv_gpio.h"
#include <stdio.h>

#define LED1_DEFAULT_PERIOD  (40U)
#define LED1_DEFAULT_TIMES   (5U)
#define LED1_DEFAULT_DUTY    (0.5f)

gpio_control_t led1;

static void GpioControlWork(gpio_control_t *handle);

/**
 * @brief 初始化 GPIO 闪烁控制对象。
 * @param handle GPIO 控制对象指针。
 * @retval 无。
 */

void GpioControlInit(gpio_control_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    handle->port = GPIOB;
    handle->pin = GPIO_PIN_2;
    handle->level = GPIO_HIGH;
    handle->period = LED1_DEFAULT_PERIOD;
    handle->light_on_percent = LED1_DEFAULT_DUTY;
    handle->times = LED1_DEFAULT_TIMES;
    handle->reset = 1U;
}

/**
 * @brief 执行单个 GPIO 控制对象的闪烁状态机。
 * @param handle GPIO 控制对象指针。
 * @retval 无。
 */

static void GpioControlWork(gpio_control_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    if (handle->reset == 1U)
    {
        handle->reset = 0U;
        handle->cnt = 0U;
        handle->times_cnt = 0U;
        handle->end = 0U;
    }

    if (handle->times_cnt >= handle->times)
    {
        handle->end = 1U;
        if (handle->level == GPIO_HIGH)
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_SET);
        }
        return;
    }

    handle->cnt++;

    if ((float)handle->cnt <= ((float)handle->period * handle->light_on_percent))
    {
        if (handle->level == GPIO_HIGH)
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_RESET);
        }
    }
    else if (handle->cnt < handle->period)
    {
        if (handle->level == GPIO_HIGH)
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(handle->port, handle->pin, GPIO_PIN_SET);
        }
    }
    else
    {
        handle->cnt = 0U;
        handle->times_cnt++;
    }
}

/**
 * @brief 配置 LED 闪烁周期、点亮占空比和重复次数。
 * @param handle GPIO 控制对象指针。
 * @param period 闪烁周期，单位为调度 tick。
 * @param light_on_percent 有效电平占空比，范围 0.0f 到 1.0f。
 * @param times 需要执行的闪烁周期次数。
 * @retval 无。
 */

void LedSetup(gpio_control_t *handle, uint32_t period, float light_on_percent, uint16_t times)
{
    if (handle == NULL)
    {
        return;
    }

    handle->period = period;
    handle->light_on_percent = light_on_percent;
    handle->reset = 1U;
    handle->times = times;
}

/**
 * @brief LED 周期任务入口。
 * @param 无。
 * @retval 无。
 */

void TaskLedProc(void)
{
    GpioControlWork(&led1);
}

