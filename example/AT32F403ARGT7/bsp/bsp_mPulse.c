/**
 * @file bsp_mPulse.c
 * @author 19816
 * @brief 脉冲输出板级支持包源文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include "at32f403a_407.h"
#include "bsp_mPulse.h"

/**
 * @brief 初始化板载脉冲输出（PB5 -> TMR3_CH2）
 * @param 无
 * @retval 无
 */
void BSP_MPulse_Init(void)
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_oc_init;

    /* 使能 GPIOB、TMR3、IOMUX 外设时钟 */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    /* 将 PB5 复用到 TMR3_CH2 输出通道 */
    gpio_pin_remap_config(TMR3_GMUX_0010, TRUE);

    /* 配置 PB5 复用输出模式 */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pins = GPIO_PINS_5;
    gpio_init(GPIOB, &gpio_init_struct);

    /* 配置 TMR3 计数基准：240MHz / (1199 + 1) = 200kHz */
    tmr_base_init(TMR3, 1199U, 0U);
    tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);

    /* 配置 PWM 通道模式和输出极性 */
    tmr_output_default_para_init(&tmr_oc_init);
    tmr_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_init.oc_output_state = TRUE;
    tmr_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_2, &tmr_oc_init);

    /* 设置占空比为 50%（600 / 1200） */
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_2, 600U);

    /* 启动定时器输出脉冲 */
    tmr_counter_enable(TMR3, TRUE);
}

