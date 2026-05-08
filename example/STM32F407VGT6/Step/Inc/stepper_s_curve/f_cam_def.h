/**
 * @file f_cam_def.h
 * @author Ws
 * @brief 步进电机 S 曲线配置入口头文件。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef F_CAM_DEF_H
#define F_CAM_DEF_H

#include "s_curve_params.h"
#include "func_scrv.h"
#include "func_plus.h"

#ifndef STEPPER_TIM_FREQ
#define STEPPER_TIM_FREQ    (1000000UL)
#endif

#ifndef STEPPER_SYS_FREQ
#define STEPPER_SYS_FREQ    STEPPER_TIM_FREQ
#endif

#ifndef STEPPER_CH_NUM
#define STEPPER_CH_NUM      (1U)
#endif

#ifndef SYS_FREQ
#define SYS_FREQ            STEPPER_TIM_FREQ
#endif

#ifndef BITBAND_PERIPH_BASE
#define BITBAND_PERIPH_BASE  (0x40000000UL)
#endif

#ifndef BITBAND_ALIAS_BASE
#define BITBAND_ALIAS_BASE   (0x42000000UL)
#endif

#ifndef HWREGBITW
#define HWREGBITW(addr, bit) \
    (*((volatile unsigned long *)(BITBAND_ALIAS_BASE + \
    (((unsigned long)(addr) - BITBAND_PERIPH_BASE) << 5) + ((bit) << 2))))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* F_CAM_DEF_H */

