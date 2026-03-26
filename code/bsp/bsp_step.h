#ifndef __BSP_STEP_H
#define __BSP_STEP_H

#include "drv_step.h"

void bsp_step_init(void);

void tmr_pwm_start_dma(tmr_type *tmr,
                       uint8_t channel,
                       uint16_t *buf,
                       uint16_t len);

void Step_DMA_IRQHandler(stepTypedef *hstep);

extern stepTypedef step2;
extern stepTypedef step3;

#endif
