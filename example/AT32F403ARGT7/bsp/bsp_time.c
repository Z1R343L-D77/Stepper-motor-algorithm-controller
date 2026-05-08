/**
 * @file bsp_time.c
 * @author 19816
 * @brief 系统 1ms 时基源文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include "bsp_time.h"
#include "bsp_tpf.h"
#include "at32f403a_407_clock.h"

#define BSP_TIMER_CLOCK_HZ      (240000000UL)
#define BSP_TIMER_COUNTER_HZ    (1000000UL)
#define BSP_TIMER_TICK_HZ       (1000UL)
#define BSP_TIMER_DIV_1MHZ      ((BSP_TIMER_CLOCK_HZ / BSP_TIMER_COUNTER_HZ) - 1U)
#define BSP_TIMER_PERIOD_1MS    ((BSP_TIMER_COUNTER_HZ / BSP_TIMER_TICK_HZ) - 1U)


/**
 * @brief 初始化 TMR1 为 1ms 时基定时器
 * @param 无
 * @retval 无
 */
void BSP_Timer_Init(void)
{
    /* 使能 TMR1 时钟，准备配置时基定时器 */
    crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

    /* 配置 1ms 周期：240MHz / (239+1) = 1MHz，1MHz / (999+1) = 1kHz */
    tmr_base_init(TMR1, BSP_TIMER_PERIOD_1MS, BSP_TIMER_DIV_1MHZ);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
    tmr_counter_value_set(TMR1, 0U);
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);

    /* 使能中断并启动定时器 */
    nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0U, 0U);
    tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
    tmr_counter_enable(TMR1, TRUE);
}


/**
 * @brief 步进主流程定时器初始化占位函数
 * @param 无
 * @retval 无
 */
void BSP_TimerStepMain_Init(void)
{
    /* 该函数当前作为扩展预留，保留空实现 */
}

/**
 * @brief TMR1 溢出中断服务函数
 * @param 无
 * @retval 无
 */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    /* 仅在溢出标志有效时推进系统节拍和调度节拍 */
    if (tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
    {
        TP_TickUpdate(1U);
        tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    }
}

