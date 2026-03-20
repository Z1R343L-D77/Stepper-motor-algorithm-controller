/**
 * @file bsp_step.c
 * @author {Z1R343L} (Z1R343L@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "stdio.h"
#include "bsp_step.h"

/**
 * @brief 步进电机DMA传输完成中断处理
 * @param tmr 定时器
 * @retval none
 */
void Step_DMA_IRQHandler(stepTypedef *hstep)
{
  if (hstep == &step3)
  {
    if (step3.state != Stop)  /* 未运动到目标脉冲数量 */
    {
      step3.flag = 1;
    }
    else                      /* 已运动到目标脉冲数量 */
    { 
      Step_Abort(hstep);    /* 步进电机强制停止 */
      Step_Unlock(&step3);  /* 解锁，允许中断控制 */
      step3_move_isr();     /* 触发中断控制 */
    }
  }else if (hstep == &step2) 
  {
    if (step2.state != Stop)  /* 未运动到目标脉冲数量 */
    {
      step2.flag = 1;
    }
    else                      /* 已运动到目标脉冲数量 */
    { 
      Step_Abort(hstep);    /* 步进电机强制停止 */
      Step_Unlock(&step2);  /* 解锁，允许中断控制 */      
      step2_move_isr();     /* 触发中断控制 */
    }
  }
}

/**
 * @brief DMA1_Channel6_IRQHandler
 * @param none
 * @retval none
 */ 
void DMA1_Channel6_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT6_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_FDT6_FLAG);
    Step_DMA_IRQHandler(&step2);
  }
}

/**
 * @brief DMA1_Channel7_IRQHandler
 * @param none
 * @retval none
 */     
void DMA1_Channel7_IRQHandler(void)
{
  if(dma_flag_get(DMA1_FDT7_FLAG) != RESET)
  {
    dma_flag_clear(DMA1_FDT7_FLAG);
    Step_DMA_IRQHandler(&step3);
  }
}

/**
 * @brief 启动DMA
 * @param tmr 定时器
 * @param channel 定时器通道
 * @param buf 缓冲区
 * @param len 缓冲区长度
 * @retval none
 */
void tmr_pwm_start_dma(tmr_type *tmr,uint8_t channel,uint16_t *buf,uint16_t len)
{
  dma_init_type dma_init_struct;
  dma_channel_type *dma_ch = NULL;
  uint32_t dma_req = 0;

  /* 根据定时器+通道选择DMA */
  if(tmr == TMR2 && channel == TMR_SELECT_CHANNEL_2)
  {
    dma_ch = DMA1_CHANNEL6;
    dma_req = TMR_C2_DMA_REQUEST;
  }
  else if(tmr == TMR5 && channel == TMR_SELECT_CHANNEL_3)
  {
    dma_ch = DMA1_CHANNEL7;
      dma_req = TMR_C3_DMA_REQUEST;
  }

  dma_reset(dma_ch);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)buf;
  /* 写PSC(div)改变频率 */
  dma_init_struct.peripheral_base_addr = (uint32_t)&tmr->div;
  dma_init_struct.buffer_size = len;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init(dma_ch, &dma_init_struct);
  /* DMA完成中断 */
  dma_interrupt_enable(dma_ch, DMA_FDT_INT, TRUE);
  /* 启动DMA */
  dma_channel_enable(dma_ch, TRUE);
  /* 开启定时器DMA请求 */
  tmr_dma_request_enable(tmr, (tmr_dma_request_type)dma_req, TRUE);
  tmr_counter_enable(tmr, TRUE);
}

/**
 * @brief 初始化步进电机方向引脚
 * @param none
 * @retval none
 */
static void step_dir_init(void)
{
  gpio_init_type gpio_init_struct;
  
  /* 使能 GPIOA 和 IOMUX 时钟 */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
  
  /* 配置 PA3 为推挽输出 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);
}
/**
 * @brief 初始化步进电机定时器
 * @param none
 * @retval none
 */
static void step_tim_init(void)
{
  gpio_init_type gpio_init_struct;
  tmr_output_config_type tmr_oc_init;

  /* 时钟使能 */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

  /* PA0 PA1 PA2 -> TIM8_CH1 TIM5_CH2 TIM2_CH3 */
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_1 | GPIO_PINS_2 ;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_init_struct);

  /* 定时器基础 */
  tmr_base_init(TMR2,999,2399);         /* 步进电机2定时器启动频率100hz */
  tmr_base_init(TMR5,999,2399);         /* 步进电机5定时器启动频率100hz */
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);

  /* PWM配置 */
  tmr_output_default_para_init(&tmr_oc_init);
  tmr_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_oc_init.oc_output_state = TRUE;
  tmr_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;

  /* CH2 PA1 */
  tmr_output_channel_config(TMR2,TMR_SELECT_CHANNEL_2,&tmr_oc_init);
  tmr_channel_value_set(TMR2,TMR_SELECT_CHANNEL_2,500);
  /* CH3 PA2 */
  tmr_output_channel_config(TMR5,TMR_SELECT_CHANNEL_3,&tmr_oc_init);
  tmr_channel_value_set(TMR5,TMR_SELECT_CHANNEL_3,500);

  /* 启动定时器 */
  tmr_counter_enable(TMR2, TRUE);
  tmr_counter_enable(TMR5, TRUE);
}

/**

 * @brief 初始化步进电机DMA
 * @param none
 * @retval none
 */
static void step_dma_init(void)
{
  /* DMA时钟 */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

  /* 绑定DMA request */
  dma_flexible_config(DMA1, FLEX_CHANNEL6, DMA_FLEXIBLE_TMR2_CH2);
  dma_flexible_config(DMA1, FLEX_CHANNEL7, DMA_FLEXIBLE_TMR5_CH3);

  /* NVIC */
  nvic_irq_enable(DMA1_Channel6_IRQn, 1, 0);
  nvic_irq_enable(DMA1_Channel7_IRQn, 1, 0);
}

/**
 * @brief 初始化步进电机
 * @param none
 * @retval none
 */
void bsp_step_init(void)
{
  step_dir_init();
  step_tim_init();
  step_dma_init();
}
