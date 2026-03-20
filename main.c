/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  * 
  * author: Z1R343L
  **************************************************************************
  */
  
/* #define AT32F403AVGT7,USE_STDPERIPH_DRIVER,AT_START_F403A_V1  */

#include "drv_step.h"
#include "bsp_step.h"

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  //***** step start*******//
  bsp_step_init();                                                                /* 初始化步进电机GPIO */
  Step_Init(&step2,TMR2,TMR_SELECT_CHANNEL_2,GPIOB,GPIO_PINS_1,500,8000,500);     /* 步进电机配置初始化 */
  Step_Init(&step3,TMR5,TMR_SELECT_CHANNEL_3,GPIOA,GPIO_PINS_3,500,8000,500);     /* 步进电机初始化 */
  step_move_start_pwm(&step2, 6400,DIR_LEFT,Decelerate_USE);
  step_move_start_pwm(&step3, 32000,DIR_LEFT,Decelerate_USE);
  //***** step end*******//
  while(1)
  {
    os_step_move_scan();
  }
}

/**
 * @brief  步进电机扫描任务函数
 * @param  None
 * @retval None
 * @note   此函数在主循环中调用，用于扫描步进电机的脉冲缓冲区
 */
void os_step_move_scan(void)
{
    if(step2.flag)
    {
        if(Step_IsBuffRdy(&step2))
        {
            tmr_pwm_start_dma(step2.tmr, step2.channel, (uint16_t *)Step_GetCurBuffer(&step2), Step_BuffUsedLength(&step2));
            Step_BufferUsed(&step2);
        }
        Step_BuffFill(&step2);
        step2.flag = 0;
    }

    if(step3.flag)
    {
        if(Step_IsBuffRdy(&step3))
        {
            tmr_pwm_start_dma(step3.tmr, step3.channel, (uint16_t *)Step_GetCurBuffer(&step3), Step_BuffUsedLength(&step3));
            Step_BufferUsed(&step3);
        }
        Step_BuffFill(&step3);
        step3.flag = 0;
    }
}

/**
 * @brief 启动步进电机 PWM 运行
 * @param hstep 步进电机控制句柄指针
 * @param stepToGo 要移动的步数
 * @param dir 方向    0: 正向  1: 反向
 * @param useDec 是否使用减速    0: 不使用  1: 使用
 */
void step_move_start_pwm(stepTypedef *hstep, uint32_t stepToGo, uint8_t dir, uint8_t useDec)
{
    Step_Prefill(hstep, stepToGo, dir, useDec);

    if(hstep == &step2)
    {
        step2.flag = 1;
    }
    else if(hstep == &step3)
    {
        step3.flag = 1;
    }
}