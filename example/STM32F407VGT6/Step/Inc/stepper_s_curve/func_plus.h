/**
 * @file func_plus.h
 * @author Ws
 * @brief 步进脉冲驱动接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef FUNC_PLUS_H
#define FUNC_PLUS_H

#include "s_curve_params.h"
#include "func_scrv.h"

#ifdef STEPPER_DRIVER_MAIN
#define STEPPER_DRIVER_EXT
#else
#define STEPPER_DRIVER_EXT  extern
#endif

#ifndef STEPPER_TIM_FREQ
#define STEPPER_TIM_FREQ     (1000000UL)
#endif

#ifndef STEPPER_SYS_FREQ
#define STEPPER_SYS_FREQ     STEPPER_TIM_FREQ
#endif

#ifndef STEPPER_CH_NUM
#define STEPPER_CH_NUM       (1U)
#endif

#ifndef SYS_FREQ
#define SYS_FREQ             STEPPER_TIM_FREQ
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

#define FUNC_PLUS_FREQ_MIN   (30.0f)
#define FUNC_PLUS_FREQ_MAX   (100000.0f)
#define FUNC_PLUS_STD_PERIOD (SYS_FREQ / 1000U)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DIR_FORWARD = 0,
    DIR_INVERSE
} func_plus_dir_e;

typedef enum
{
    FUNC_PLUS_MODE_C = 0,
    FUNC_PLUS_MODE_L,
    FUNC_PLUS_MODE_SLV
} func_plus_mode_e;

typedef struct
{
    float plus_dist;
    float j;
    float amax;
    float vmax;
    float *phost_v;
    volatile unsigned long *p_dir;
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
    volatile unsigned long *p_tmr_cc;
    volatile unsigned long *p_ocx_mode0;
    volatile unsigned long *p_dir;
    unsigned int rev0;
    unsigned char dir_forward;
    unsigned char b_fl;
    unsigned char b_org;
    unsigned char b_run;
    func_scrv_t scrv;
} func_plus_ch_t;

typedef struct
{
    func_plus_ch_t ch[STEPPER_CH_NUM];
} func_plus_t;

/**
 * @brief 初始化单个步进脉冲通道参数。
 * @param handle 步进脉冲通道指针。
 * @param init 通道初始化参数指针。
 * @retval 无。
 */
void FuncPlusChInit(func_plus_ch_t *handle, func_plus_ch_init_t *init);

/**
 * @brief 初始化步进脉冲驱动模块。
 * @param handle 步进脉冲驱动主对象指针。
 * @retval 无。
 */
void FuncPlusInit(func_plus_t *handle);

/**
 * @brief 根据目标速度更新通道运行状态。
 * @param handle 步进脉冲通道指针。
 * @param v 目标速度。
 * @retval 无。
 */
void FuncPlusChVUpdate(func_plus_ch_t *handle, float v);

/**
 * @brief 1ms 周期任务，驱动各通道 S 曲线状态机。
 * @param 无。
 * @retval 无。
 */
void FuncPlusTick(void);

/**
 * @brief 步进脉冲捕获比较中断处理核心。
 * @param handle 步进脉冲通道指针。
 * @retval 无。
 */
void FuncPlusIt(func_plus_ch_t *handle);

/**
 * @brief 按 S 曲线执行指定距离的定长运动。
 * @param handle 步进脉冲通道指针。
 * @param vc 巡航速度。
 * @param v1 末速度。
 * @param l 目标距离。
 * @param dir 运动方向。
 * @param cb 启动回调。
 * @retval 1 成功。
 * @retval 0 失败。
 */
unsigned char FuncPlusSL(func_plus_ch_t *handle, float vc, float v1,
                            float l, func_plus_dir_e dir, func_scrv_cb_t cb);

/**
 * @brief 按给定初末速度、距离和时间生成 S 曲线运动。
 * @param handle 步进脉冲通道指针。
 * @param v0 初始速度。
 * @param v1 末速度。
 * @param l 距离。
 * @param t 时间。
 * @param dir 运动方向。
 * @retval 1 成功。
 * @retval 0 失败。
 */
unsigned char FuncPlusSTlv(func_plus_ch_t *handle, float v0, float v1,
                              float l, float t, func_plus_dir_e dir);

/**
 * @brief 按 S 曲线平滑切换到目标连续速度。
 * @param handle 步进脉冲通道指针。
 * @param v 目标速度。
 * @param dir 运动方向。
 * @param cb 启动回调。
 * @retval 1 成功。
 * @retval 0 失败。
 */
unsigned char FuncPlusSCv(func_plus_ch_t *handle, float v,
                             func_plus_dir_e dir, func_scrv_cb_t cb);

/**
 * @brief 将通道切换为从属速度模式。
 * @param handle 步进脉冲通道指针。
 * @retval 1 成功。
 */
unsigned char FuncPlusSSlv(func_plus_ch_t *handle);

/**
 * @brief 查询通道是否仍处于忙状态。
 * @param handle 步进脉冲通道指针。
 * @retval 1 忙。
 * @retval 0 空闲。
 */
unsigned char FuncPlusSBusy(func_plus_ch_t *handle);

/**
 * @brief 查询 S 曲线指令缓冲区是否为空。
 * @param handle 步进脉冲通道指针。
 * @retval 1 空。
 * @retval 0 非空。
 */
unsigned char FuncPlusSBufEmpty(func_plus_ch_t *handle);

/**
 * @brief 清零通道当前脉冲计数。
 * @param handle 步进脉冲通道指针。
 * @retval 无。
 */
void FuncPlusSClrPlus(func_plus_ch_t *handle);

/**
 * @brief 初始化硬件 MSP 资源。
 * @param handle 步进脉冲驱动主对象指针。
 * @retval 无。
 */
void FuncPlusMspInit(func_plus_t *handle);

STEPPER_DRIVER_EXT func_plus_t g_func_plus;

#ifdef __cplusplus
}
#endif

#endif /* FUNC_PLUS_H */

