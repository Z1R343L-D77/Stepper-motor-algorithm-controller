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
#include "at32f403a_407_clock.h"
#include "at32_system_delay.h"

#include "drv_gpio.h"
#include "drv_goseiko.h"

#include "bsp_time.h"
#include "bsp_led.h"
#include "bsp_mPulse.h"

#include "bsp_tpf.h"
#include "stepper_app.h"

static tptask_t task_stepper;

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  system_clock_config();
  StepperApp_Init(&g_func_plus);
  BSP_MPulse_Init();
  /* step end */
  BSP_Timer_Init();
  TP_TaskInit(&task_stepper, 1U, StepperApp_Tick1ms);
  while(1)
  {
    TP_TaskHandler();
  }
}

