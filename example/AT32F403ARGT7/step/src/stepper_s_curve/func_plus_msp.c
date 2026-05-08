/**
 * @file func_plus_msp.c
 * @author 19816
 * @brief 步进脉冲硬件适配层源文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#include <stdio.h>
#include "stepper_app_def.h"
#include "at32f403a_407.h"

#define TMR5_C1OCTRL_BIT0      HWREGBITW((TMR5_BASE + 0x18UL), 4)
#define TMR5_C1DT_ADDR         ((volatile unsigned long *)(TMR5_BASE + 0x34UL))
#define TMR_C1OCTRL_MASK       (0x7UL << 4)
#define TMR_C1OCTRL_LOW        ((uint32_t)TMR_OUTPUT_CONTROL_LOW << 4)
#define TMR5_TIM_DIV_1MHZ      (239U)
#define TMR5_TIM_PERIOD_MAX    (0xFFFFFFFFUL)
#define TMR5_FIRST_COMPARE     (1000U)

/**
  * @brief  初始化 AT32F403A 步进脉冲硬件资源。
  * @param  handle 步进脉冲驱动对象指针。
  * @retval 无
  */
void FuncPlus_MspInit(func_plus_t* handle)
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_oc_init;

    if (handle == NULL)
    {
        return;
    }

    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_0;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOA, &gpio_init_struct);

    handle->ch[0].tmr_cc_ptr = TMR5_C1DT_ADDR;
    handle->ch[0].oc_mode0_ptr = &TMR5_C1OCTRL_BIT0;

    tmr_counter_enable(TMR5, FALSE);
    tmr_32_bit_function_enable(TMR5, TRUE);
    tmr_base_init(TMR5, TMR5_TIM_PERIOD_MAX, TMR5_TIM_DIV_1MHZ);
    tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);
    tmr_counter_value_set(TMR5, 0U);

    tmr_output_default_para_init(&tmr_oc_init);
    tmr_oc_init.oc_mode = TMR_OUTPUT_CONTROL_LOW;
    tmr_oc_init.oc_output_state = TRUE;
    tmr_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_1, &tmr_oc_init);
    tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_1, TMR5_FIRST_COMPARE);

    TMR5->cm1 = (TMR5->cm1 & ~TMR_C1OCTRL_MASK) | TMR_C1OCTRL_LOW;
    tmr_interrupt_enable(TMR5, TMR_C1_INT, TRUE);
    tmr_flag_clear(TMR5, TMR_C1_FLAG);

    nvic_irq_enable(TMR5_GLOBAL_IRQn, 0, 0);
    tmr_counter_enable(TMR5, TRUE);
}

/**
  * @brief  TMR5 捕获比较中断服务函数。
  * @param  无
  * @retval 无
  */
void TMR5_GLOBAL_IRQHandler(void)
{
    if (tmr_flag_get(TMR5, TMR_C1_FLAG) != RESET)
    {
        tmr_flag_clear(TMR5, TMR_C1_FLAG);
        FuncPlus_It(&g_func_plus.ch[0]);
    }
}

