/**
 * @file stepper_app_def.h
 * @author 19816
 * @brief 步进电机 S 曲线配置入口头文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#ifndef STEPPER_APP_DEF_H
#define STEPPER_APP_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "s_curve_params.h"
#include "func_scrv.h"
#include "func_plus.h"

#ifndef STEPPER_TIM_FREQ
#define STEPPER_TIM_FREQ    1000000UL
#endif

#ifndef STEPPER_SYS_FREQ
#define STEPPER_SYS_FREQ    STEPPER_TIM_FREQ
#endif

#ifndef STEPPER_CH_NUM
#define STEPPER_CH_NUM      1
#endif

#ifndef SYS_FREQ
#define SYS_FREQ            STEPPER_TIM_FREQ
#endif

#ifndef HWREGBITW
#define BITBAND_PERIPH_BASE  0x40000000UL
#define BITBAND_ALIAS_BASE   0x42000000UL
#define HWREGBITW(addr, bit) \
    (*((volatile unsigned long *)(BITBAND_ALIAS_BASE + \
     (((unsigned long)(addr) - BITBAND_PERIPH_BASE) << 5) + ((bit) << 2))))
#endif

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_APP_DEF_H */

