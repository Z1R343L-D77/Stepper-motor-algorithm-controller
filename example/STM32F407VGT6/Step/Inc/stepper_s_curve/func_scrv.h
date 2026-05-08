/**
 * @file func_scrv.h
 * @author Ws
 * @brief S 曲线状态机接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef FUNC_SCRV_H
#define FUNC_SCRV_H

#include "s_curve_params.h"

#ifdef FUNC_SCRV_MAIN
#define FUNC_SCRV_EXT
#else
#define FUNC_SCRV_EXT  extern
#endif

#define FUNC_SCRV_T_UNI  (0.001f)
#define SCRV_CB0         (&FuncScrvCb0)
#define SCRV_CB1         (&FuncScrvCb1)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FUNC_SCRV_MODE_S = 0,
    FUNC_SCRV_MODE_SLV
} func_scrv_mode_e;

typedef enum
{
    FUNC_SCRV_STD = 0,
    FUNC_SCRV_JA1,
    FUNC_SCRV_JA0,
    FUNC_SCRV_JAN1,
    FUNC_SCRV_C,
    FUNC_SCRV_JDN1,
    FUNC_SCRV_JD0,
    FUNC_SCRV_JD1,
    FUNC_SCRV_END
} func_scrv_step_e;

typedef unsigned char (*func_scrv_cb_t)(void);

typedef struct
{
    unsigned char b_lock;
    unsigned char b_new_sp;
    unsigned char b_slv;
    func_scrv_cb_t cb;
    func_s_params_t sp;
} func_scrv_buf_t;

typedef struct
{
    func_scrv_buf_t buf;

    unsigned char step;
    unsigned char err_cnt;
    unsigned char b_slv;
    float *phost_v;

    float j;
    float amax;
    float vmax;
    float freq_min;

    float a;
    float v;
    float t;
    float tx;

    float t_last;
    float a_last;
    float v_last;

    func_s_params_t sp;
} func_scrv_t;

/**
 * @brief 执行一次 S 曲线状态机计算。
 * @param handle S 曲线状态机对象指针。
 * @retval 当前目标速度。
 */
float FuncScrvTick(func_scrv_t *handle);

/**
 * @brief 默认等待回调。
 * @param 无。
 * @retval 0 表示暂不启动。
 */
unsigned char FuncScrvCb0(void);

/**
 * @brief 默认立即执行回调。
 * @param 无。
 * @retval 1 表示允许启动。
 */
unsigned char FuncScrvCb1(void);

/**
 * @brief 查询参数缓冲区是否为空。
 * @param handle S 曲线状态机对象指针。
 * @retval 1 空。
 * @retval 0 非空。
 */
unsigned char FuncScrvBufIsEmpty(func_scrv_t *handle);

/**
 * @brief 锁定参数缓冲区。
 * @param handle S 曲线状态机对象指针。
 * @retval 无。
 */
void FuncScrvBufLock(func_scrv_t *handle);

/**
 * @brief 解锁参数缓冲区并提交新参数。
 * @param handle S 曲线状态机对象指针。
 * @param b_slv 是否从属模式。
 * @param cb 启动条件回调。
 * @retval 无。
 */
void FuncScrvBufUnlock(func_scrv_t *handle, unsigned char b_slv, func_scrv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* FUNC_SCRV_H */

