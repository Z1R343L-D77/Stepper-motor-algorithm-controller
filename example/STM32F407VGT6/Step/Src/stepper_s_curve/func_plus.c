/**
 * @file func_plus.c
 * @author Ws
 * @brief 步进脉冲驱动核心实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#define STEPPER_DRIVER_MAIN
#include "func_plus.h"
#include <math.h>
#include <stdio.h>

/**
 * @brief 根据目标频率更新通道定时器比较增量。
 * @param handle 步进脉冲通道指针。
 * @param freq 目标脉冲频率，单位 Hz。
 * @retval 无。
 * @note 频率会被限制在 FUNC_PLUS_FREQ_MIN 到 FUNC_PLUS_FREQ_MAX 范围内。
 */
static void FuncPlusChFreqUpdate(func_plus_ch_t *handle, float freq)
{
    float f;

    if (freq < FUNC_PLUS_FREQ_MIN)
    {
        freq = FUNC_PLUS_FREQ_MIN;
    }
    else if (freq > FUNC_PLUS_FREQ_MAX)
    {
        freq = FUNC_PLUS_FREQ_MAX;
    }
    else
    {
        /* 频率在允许范围内时保持原值。 */
    }

    handle->freq = freq;
    f = (float)(SYS_FREQ) * 256.0f / freq;
    handle->cmp_incx256 = (unsigned int)(f + 0.5f);
}

/**
 * @brief 根据目标速度更新通道脉冲频率。
 * @param handle 步进脉冲通道指针。
 * @param v 目标线速度。
 * @retval 无。
 */
void FuncPlusChVUpdate(func_plus_ch_t *handle, float v)
{
    float freq;

    handle->v = v;
    freq = v / handle->plus_dist;

    if (freq < handle->scrv.freq_min)
    {
        freq = handle->scrv.freq_min;
    }

    FuncPlusChFreqUpdate(handle, freq);
}

/**
 * @brief 初始化单个步进脉冲通道的软件参数。
 * @param handle 步进脉冲通道指针。
 * @param init 通道初始化参数指针。
 * @retval 无。
 */
void FuncPlusChInit(func_plus_ch_t *handle, func_plus_ch_init_t *init)
{
    float f;

    handle->plus_dist = init->plus_dist;

    handle->scrv.b_slv = 0U;
    handle->scrv.amax = init->amax;
    handle->scrv.j = init->j;
    handle->scrv.vmax = init->vmax;
    handle->scrv.phost_v = init->phost_v;
    handle->scrv.buf.cb = &FuncScrvCb0;

    f = init->plus_dist;
    f = f * 6.0f / init->j;
    f = powf(f, 0.33333333f);
    handle->scrv.freq_min = 1.0f / f;

    handle->dir_forward = init->dir_forward;

    if (init->p_dir != NULL)
    {
        handle->p_dir = init->p_dir;
    }
    else
    {
        handle->p_dir = (volatile unsigned long *)&handle->rev0;
    }

    *(handle->p_dir) = handle->dir_forward;

    FuncPlusChVUpdate(handle, 0.0f);
}

/**
 * @brief 初始化步进脉冲驱动模块。
 * @param handle 步进脉冲驱动主对象指针。
 * @retval 无。
 */
void FuncPlusInit(func_plus_t *handle)
{
    FuncPlusMspInit(handle);
}

/**
 * @brief 步进脉冲捕获比较中断处理核心。
 * @param handle 步进脉冲通道指针。
 * @retval 无。
 */
void FuncPlusIt(func_plus_ch_t *handle)
{
    unsigned int cap;

    cap = *(handle->p_tmr_cc);

    if (*(handle->p_ocx_mode0) != 0U)
    {
        *handle->p_ocx_mode0 = 0U;
        cap += (handle->cmp_inc_accx256 >> 9);

        if (handle->b_run != 0U)
        {
            if (*(handle->p_dir) == handle->dir_forward)
            {
                handle->plus_cur++;
            }
            else
            {
                handle->plus_cur--;
            }
        }
    }
    else
    {
        if (handle->b_fl == 0U)
        {
            handle->b_run = (unsigned char)(handle->v != 0.0f);
        }
        else
        {
            if (handle->plus_cur == handle->plus_tag)
            {
                handle->b_run = 0U;
            }
            else if (handle->v != 0.0f)
            {
                handle->b_run = 1U;
            }
            else
            {
                /* 定长模式且速度为 0 时保持当前状态。 */
            }
        }

        if (handle->b_run != 0U)
        {
            *handle->p_ocx_mode0 = 1U;
            handle->cmp_inc_accx256 &= 0x1FFU;
            handle->cmp_inc_accx256 += handle->cmp_incx256;
            cap += (handle->cmp_inc_accx256 >> 9);
        }
        else
        {
            cap += FUNC_PLUS_STD_PERIOD;
        }
    }

    *(handle->p_tmr_cc) = cap;
}

/**
 * @brief 1ms 周期任务，驱动各通道 S 曲线状态机。
 * @param 无。
 * @retval 无。
 */
void FuncPlusTick(void)
{
    unsigned int i;
    float f;
    func_plus_ch_t *handle;

    for (i = 0U; i < STEPPER_CH_NUM; i++)
    {
        handle = &g_func_plus.ch[i];
        f = FuncScrvTick(&handle->scrv);
        FuncPlusChVUpdate(handle, f);
    }
}

/**
 * @brief 根据运动方向和距离计算定长运动目标脉冲数。
 * @param handle 步进脉冲通道指针。
 * @param dir 运动方向。
 * @param l 目标运动距离。
 * @retval 无。
 */
static void FuncPlusCalcTag(func_plus_ch_t *handle, func_plus_dir_e dir, float l)
{
    float f;
    unsigned int i;

    f = l / handle->plus_dist;
    i = (unsigned int)f;

    if (i != 0U)
    {
        if (dir == DIR_FORWARD)
        {
            i = handle->plus_cur + i;
        }
        else
        {
            i = handle->plus_cur - i;
        }

        handle->plus_tag = i;
        handle->b_fl = 1U;
    }
    else
    {
        handle->b_fl = 0U;
    }
}

/**
 * @brief 按 S 曲线执行指定距离的定长运动。
 * @param handle 步进脉冲通道指针。
 * @param vc 巡航速度，传入 0 时使用通道最大速度。
 * @param v1 运动结束速度。
 * @param l 目标运动距离。
 * @param dir 运动方向。
 * @param cb 启动回调。
 * @retval 1 指令写入成功。
 * @retval 0 当前状态不允许写入该运动指令。
 */
unsigned char FuncPlusSL(func_plus_ch_t *handle, float vc, float v1,
                            float l, func_plus_dir_e dir, func_scrv_cb_t cb)
{
    func_s_tlv_in_t input;
    func_s_params_t params;
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : (unsigned char)(!handle->dir_forward);

    if (((handle->scrv.v != 0.0f) && (*(handle->p_dir) != b_dir)) || (l < 0.0f))
    {
        handle->scrv.err_cnt++;
        return 0U;
    }

    FuncScrvBufLock(&handle->scrv);
    *(handle->p_dir) = b_dir;

    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.j = handle->scrv.j;
    input.l = l;
    input.v0 = handle->scrv.v;
    input.vc = (vc > 0.0f) ? vc : handle->scrv.vmax;
    input.v1 = v1;
    input.t = 0.0f;

    FuncSTlv(&params, &input);
    handle->scrv.buf.sp = params;

    if (params.v1 == 0.0f)
    {
        FuncPlusCalcTag(handle, dir, l);
    }
    else
    {
        handle->b_fl = 0U;
    }

    FuncScrvBufUnlock(&handle->scrv, 0U, cb);
    return 1U;
}

/**
 * @brief 按给定初末速度、距离和时间生成 S 曲线运动。
 * @param handle 步进脉冲通道指针。
 * @param v0 初始速度。
 * @param v1 末速度。
 * @param l 目标运动距离。
 * @param t 目标运动时间。
 * @param dir 运动方向。
 * @retval 1 指令写入成功。
 * @retval 0 当前状态不允许写入该运动指令。
 */
unsigned char FuncPlusSTlv(func_plus_ch_t *handle, float v0, float v1,
                              float l, float t, func_plus_dir_e dir)
{
    func_s_params_t params;
    func_s_tlv_in_t input;
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : (unsigned char)(!handle->dir_forward);

    if (((handle->scrv.v != 0.0f) && (*(handle->p_dir) != b_dir)) || (l < 0.0f))
    {
        handle->scrv.err_cnt++;
        return 0U;
    }

    FuncScrvBufLock(&handle->scrv);
    *(handle->p_dir) = b_dir;
    handle->b_fl = 0U;

    input.j = handle->scrv.j;
    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.l = l;
    input.t = t;
    input.v0 = v0;
    input.v1 = v1;
    if (input.t == 0.0f)
    {
        input.vc = handle->scrv.vmax;
    }

    FuncSTlv(&params, &input);
    handle->scrv.buf.sp = params;

    if ((params.v1 == 0.0f) && (input.l != 0.0f))
    {
        FuncPlusCalcTag(handle, dir, l);
    }
    else
    {
        handle->b_fl = 0U;
    }

    FuncScrvBufUnlock(&handle->scrv, 0U, SCRV_CB1);
    return 1U;
}

/**
 * @brief 按 S 曲线平滑切换到目标连续速度。
 * @param handle 步进脉冲通道指针。
 * @param v 目标速度。
 * @param dir 运动方向。
 * @param cb 启动回调。
 * @retval 1 指令写入成功。
 * @retval 0 当前状态不允许反向切换。
 */
unsigned char FuncPlusSCv(func_plus_ch_t *handle, float v,
                             func_plus_dir_e dir, func_scrv_cb_t cb)
{
    func_s_params_t params;
    func_s_tlv_in_t input = {0};
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : (unsigned char)(!handle->dir_forward);

    if ((handle->scrv.v != 0.0f) && (*(handle->p_dir) != b_dir))
    {
        handle->scrv.err_cnt++;
        return 0U;
    }

    FuncScrvBufLock(&handle->scrv);
    *(handle->p_dir) = b_dir;

    input.j = handle->scrv.j;
    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.v0 = handle->scrv.v;
    input.v1 = v;
    input.l = 0.0f;
    input.t = -1.0f;

    FuncSTlv(&params, &input);
    handle->scrv.buf.sp = params;
    handle->b_fl = 0U;

    FuncScrvBufUnlock(&handle->scrv, 0U, cb);
    return 1U;
}

/**
 * @brief 将通道切换为从属速度模式。
 * @param handle 步进脉冲通道指针。
 * @retval 1 模式切换成功。
 */
unsigned char FuncPlusSSlv(func_plus_ch_t *handle)
{
    FuncScrvBufLock(&handle->scrv);
    handle->b_fl = 0U;
    FuncScrvBufUnlock(&handle->scrv, 1U, SCRV_CB1);
    return 1U;
}

/**
 * @brief 查询通道是否仍处于忙状态。
 * @param handle 步进脉冲通道指针。
 * @retval 1 通道忙。
 * @retval 0 通道空闲。
 */
unsigned char FuncPlusSBusy(func_plus_ch_t *handle)
{
    unsigned char r;

    if (handle->b_fl != 0U)
    {
        r = (unsigned char)(handle->scrv.buf.b_new_sp ||
                            (handle->scrv.step != FUNC_SCRV_STD) ||
                            handle->b_run ||
                            (handle->plus_cur != handle->plus_tag));
    }
    else
    {
        r = (unsigned char)(handle->scrv.buf.b_new_sp || (handle->scrv.step != FUNC_SCRV_STD));
    }

    return r;
}

/**
 * @brief 查询 S 曲线指令缓冲区是否为空。
 * @param handle 步进脉冲通道指针。
 * @retval 1 缓冲区为空，可写入新指令。
 * @retval 0 缓冲区已有待执行指令。
 */
unsigned char FuncPlusSBufEmpty(func_plus_ch_t *handle)
{
    return (unsigned char)(handle->scrv.buf.b_new_sp == 0U);
}

/**
 * @brief 清零通道当前脉冲计数。
 * @param handle 步进脉冲通道指针。
 * @retval 无。
 */
void FuncPlusSClrPlus(func_plus_ch_t *handle)
{
    handle->plus_cur = 0U;
}

