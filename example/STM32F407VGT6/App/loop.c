/**
 * @file loop.c
 * @author Ws
 * @brief 主循环任务调度实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#include "config.h"
#include <stdio.h>

static tp_task_t task_led;
static tp_task_t task_btn;
static tp_task_t task_uart;

/**
 * @brief 初始化应用模块并运行协作式调度器。
 * @param 无。
 * @retval 无。
 * @note 应在 STM32 HAL、时钟、GPIO、UART 和 DMA 初始化完成后调用。
 */
void Loop(void)
{
    RingbufferInit(&usart_rb);
    GpioControlInit(&led1);
    ButtonInitAll();

    StepperAppInit(&g_func_plus);

    TpTaskInit(&task_led, 5U, TaskLedProc);
    TpTaskInit(&task_btn, 5U, ButtonTicks);
    TpTaskInit(&task_uart, 10U, UartProc);

    printf("\r\n=== STM32 Stepper Motor Angle Control System ===\r\n");

    for (;;)
    {
        TpTaskHandler();
    }
}

