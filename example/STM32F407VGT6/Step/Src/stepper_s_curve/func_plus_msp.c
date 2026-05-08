/**
 * @file func_plus_msp.c
 * @author Ws
 * @brief STM32F407 HAL 步进电机硬件层实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "f_cam_def.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#define TIM1_OC1M_BIT0       HWREGBITW((TIM1_BASE + 0x18UL), 4)
#define TIM1_CCR1_ADDR       ((volatile unsigned long *)(TIM1_BASE + 0x34UL))
#define TIM_OCM_MASK_CH1     (0x7UL << 4)
#define TIM_OCM_INACTIVE_CH1 (0x2UL << 4)

/**
 * @brief 初始化 STM32F407 步进脉冲硬件资源。
 * @param handle 步进脉冲驱动对象指针。
 * @retval 无。
 */

void FuncPlusMspInit(func_plus_t *handle)
{
    GPIO_InitTypeDef gpio = {0};

    if (handle == NULL)
    {
        return;
    }

    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &gpio);

    handle->ch[0].p_tmr_cc = TIM1_CCR1_ADDR;
    handle->ch[0].p_ocx_mode0 = &TIM1_OC1M_BIT0;

    if (handle->ch[0].p_dir == (volatile unsigned long *)&handle->ch[0].rev0)
    {
        handle->ch[0].p_dir = &HWREGBITW(&GPIOE->ODR, 8);
        *(handle->ch[0].p_dir) = handle->ch[0].dir_forward;
    }

    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM1->CR1 = 0U;
    TIM1->PSC = 167U;
    TIM1->ARR = 0xFFFFU;
    TIM1->CNT = 0U;

    TIM1->CCR1 = 1000U;
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_OCM_MASK_CH1) | TIM_OCM_INACTIVE_CH1;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->DIER |= TIM_DIER_CC1IE;
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0U;

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief TIM1 捕获比较中断服务函数。
 * @param 无。
 * @retval 无。
 */

void TIM1_CC_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_CC1IF) != 0U)
    {
        TIM1->SR &= ~TIM_SR_CC1IF;
        FuncPlusIt(&g_func_plus.ch[0]);
    }
}

