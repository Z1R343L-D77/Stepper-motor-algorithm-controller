/**
 * @file func_plus.h
 * @author 19816
 * @brief 步进脉冲驱动接口头文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#ifndef FUNC_PLUS_H
#define FUNC_PLUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "s_curve_params.h"
#include "func_scrv.h"

#ifdef STEPPER_DRIVER_MAIN
#define STEPPER_DRIVER_EXT
#else
#define STEPPER_DRIVER_EXT extern
#endif

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

#define FUNC_PLUS_FREQ_MIN   30.0f
#define FUNC_PLUS_FREQ_MAX   100000.0f
#define FUNC_PLUS_STD_PERIOD (SYS_FREQ / 1000)

typedef enum
{
    DIR_FORWARD = 0,
    DIR_INVERSE = 1
} func_plus_dir_e;

enum
{
    FUNC_PLUS_MODE_C,
    FUNC_PLUS_MODE_L,
    FUNC_PLUS_MODE_SLV
};

typedef struct
{
    float plus_dist;
    float j;
    float amax;
    float vmax;
    float *host_v_ptr;
    volatile unsigned long *dir_reg_ptr;
    unsigned char dir_forward;
} func_plus_ch_init_t;

typedef struct
{
    float plus_dist;
    unsigned int plus_cur;
    unsigned int plus_tag;
    float freq;
    float v;
    unsigned int cmp_incx256;
    unsigned int cmp_inc_accx256;
    volatile unsigned long *tmr_cc_ptr;
    volatile unsigned long *oc_mode0_ptr;
    volatile unsigned long *dir_reg_ptr;
    unsigned int reserve0;
    unsigned char dir_forward;
    unsigned char is_fixed_length;
    unsigned char is_origin;
    unsigned char is_running;
    func_scrv_t scrv;
} func_plus_ch_t;

typedef struct
{
    func_plus_ch_t ch[STEPPER_CH_NUM];
} func_plus_t;

void FuncPlus_ChInit(func_plus_ch_t* handle, func_plus_ch_init_t* init);
void FuncPlus_Init(func_plus_t* handle);
void FuncPlus_ChVUpdate(func_plus_ch_t* handle, float v);
void FuncPlus_Tick(void);
void FuncPlus_It(func_plus_ch_t* handle);
unsigned char FuncPlus_SL(func_plus_ch_t* handle, float vc, float v1,
                            float l, func_plus_dir_e dir, func_scrv_cb_t cb);
unsigned char FuncPlus_STlv(func_plus_ch_t* handle, float v0, float v1,
                              float l, float t, func_plus_dir_e dir);
unsigned char FuncPlus_SCv(func_plus_ch_t* handle, float v,
                             func_plus_dir_e dir, func_scrv_cb_t cb);
unsigned char FuncPlus_SSlv(func_plus_ch_t* handle);
unsigned char FuncPlus_SBusy(func_plus_ch_t* handle);
unsigned char FuncPlus_SBufEmpty(func_plus_ch_t* handle);
void FuncPlus_SClrPlus(func_plus_ch_t* handle);
void FuncPlus_MspInit(func_plus_t* handle);

STEPPER_DRIVER_EXT func_plus_t g_func_plus;

#ifdef __cplusplus
}
#endif

#endif /* FUNC_PLUS_H */

