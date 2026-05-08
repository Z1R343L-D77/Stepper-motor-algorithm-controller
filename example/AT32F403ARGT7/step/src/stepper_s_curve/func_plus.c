/**
 * @file func_plus.c
 * @author 19816
 * @brief 步进脉冲驱动核心源文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#define STEPPER_DRIVER_MAIN
#include "func_plus.h"
#include <math.h>
/**
  * @brief  根据目标频率更新通道定时器比较增量。
  * @param  handle 步进脉冲通道指针。
  * @param  freq 目标脉冲频率，单位 Hz。
  * @retval 无
  * @note   频率会被限制在 FUNC_PLUS_FREQ_MIN 到 FUNC_PLUS_FREQ_MAX 范围内。
  */
static void FuncPlus_ChFreqUpdate(func_plus_ch_t* handle, float freq)
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

    handle->freq = freq;
    f = (float)(SYS_FREQ) * 256.0f / freq;
    handle->cmp_incx256 = (unsigned int)(f + 0.5f);
}

/**
  * @brief  根据目标速度更新通道脉冲频率。
  * @param  handle 步进脉冲通道指针。
  * @param  v 目标线速度。
  * @retval 无
  */
void FuncPlus_ChVUpdate(func_plus_ch_t* handle, float v)
{
    float freq;

    handle->v = v;
    freq = v / handle->plus_dist;

    if (freq < handle->scrv.freq_min)
    {
        freq = handle->scrv.freq_min;
    }

    FuncPlus_ChFreqUpdate(handle, freq);
}

/**
  * @brief  初始化单个步进脉冲通道的软件参数。
  * @param  handle 步进脉冲通道指针。
  * @param  init 通道初始化参数指针。
  * @retval 无
  * @note   会计算 S 曲线最小频率，并设置方向输出默认状态。
  */
void FuncPlus_ChInit(func_plus_ch_t* handle, func_plus_ch_init_t* init)
{
    float f;

    handle->plus_dist = init->plus_dist;

    handle->scrv.is_slave  = 0;
    handle->scrv.amax   = init->amax;
    handle->scrv.j      = init->j;
    handle->scrv.vmax   = init->vmax;
    handle->scrv.host_v_ptr = init->host_v_ptr;
    handle->scrv.buf.cb  = &FuncScrv_Cb0;

    /* 计算最小频率：从 0 速度加速走过 1 个脉冲距离所需时间
     * l = (1/6)*j*t^3  =>  t = (6l/j)^(1/3)
     */
    f = 1.0f * init->plus_dist;
    f = f * 6.0f / init->j;
    f = powf(f, 0.33333333f);
    handle->scrv.freq_min = 1.0f / f;

    handle->dir_forward = init->dir_forward;

    if (init->dir_reg_ptr)
    {
        handle->dir_reg_ptr = init->dir_reg_ptr;
    }
    else
    {
        /* 当前未配置外部方向IO，使用软件变量保存方向状态。 */
        handle->dir_reg_ptr = (volatile unsigned long *)&handle->reserve0;
    }

    *(handle->dir_reg_ptr) = handle->dir_forward;

    FuncPlus_ChVUpdate(handle, 0);
}

/**
  * @brief  初始化步进脉冲驱动模块。
  * @param  handle 步进脉冲驱动主对象指针。
  * @retval 无
  * @note   内部调用 FuncPlus_MspInit 完成 AT32 硬件层初始化。
  */
void FuncPlus_Init(func_plus_t* handle)
{
    FuncPlus_MspInit(handle);
}

/**
  * @brief  步进脉冲捕获比较中断处理核心。
  * @param  handle 步进脉冲通道指针。
  * @retval 无
  * @note   在定时器比较中断中调用，用于生成脉冲并维护当前位置计数。
  */
void FuncPlus_It(func_plus_ch_t* handle)
{
    unsigned int cap;

    cap = *(handle->tmr_cc_ptr);

    if (*handle->oc_mode0_ptr)
    {
        /* 下降沿：关闭翻转，更新比较寄存器高位 */
        *handle->oc_mode0_ptr = 0;
        cap += (handle->cmp_inc_accx256 >> 9);

        if (handle->is_running)
        {
            if (*(handle->dir_reg_ptr) == handle->dir_forward)
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
        /* 上升沿：判断运行状态 */
        if (!handle->is_fixed_length)
        {
            /* 连续速度模式 */
            handle->is_running = (handle->v != 0);
        }
        else
        {
            /* 定长模式 */
            if (handle->plus_cur == handle->plus_tag)
            {
                handle->is_running = 0;
            }
            else if (handle->v != 0)
            {
                handle->is_running = 1;
            }
        }

        if (handle->is_running)
        {
            *handle->oc_mode0_ptr = 1;
            handle->cmp_inc_accx256 &= 0x1ff;
            handle->cmp_inc_accx256 += handle->cmp_incx256;
            cap += (handle->cmp_inc_accx256 >> 9);
        }
        else
        {
            /* 停止时以 1kHz 维持中断响应，保持状态机活跃 */
            cap += FUNC_PLUS_STD_PERIOD;
        }
    }
    *(handle->tmr_cc_ptr) = cap;
}


/**
  * @brief  1ms 周期任务，驱动各通道 S 曲线状态机。
  * @param  无
  * @retval 无
  */
void FuncPlus_Tick(void)
{
    int i;
    float f;
    func_plus_ch_t* handle;

    for (i = 0; i < STEPPER_CH_NUM; i++)
    {
        handle = &g_func_plus.ch[i];
        f = FuncScrv_Tick(&handle->scrv);
        FuncPlus_ChVUpdate(handle, f);
    }
}

/**
  * @brief  根据运动方向和距离计算定长运动目标脉冲数。
  * @param  handle 步进脉冲通道指针。
  * @param  dir 运动方向。
  * @param  l 目标运动距离。
  * @retval 无
  */
static void FuncPlus_CalcTag(func_plus_ch_t* handle, func_plus_dir_e dir, float l)
{
    float f;
    unsigned int i;

    f = l / handle->plus_dist;
    i = (unsigned int)f;

    if (i != 0)
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
        handle->is_fixed_length = 1;
    }
    else
    {
        handle->is_fixed_length = 0;
    }
}

/**
  * @brief  按 S 曲线执行指定距离的定长运动。
  * @param  handle 步进脉冲通道指针。
  * @param  vc 巡航速度，传入 0 时使用通道最大速度。
  * @param  v1 运动结束速度。
  * @param  l 目标运动距离。
  * @param  dir 运动方向。
  * @param  cb 启动回调，SCRV_CB1 表示立即执行，SCRV_CB0 表示等待触发。
  * @retval 1 指令写入成功。
  * @retval 0 当前状态不允许写入该运动指令。
  */
unsigned char FuncPlus_SL(func_plus_ch_t* handle, float vc, float v1,
                             float l, func_plus_dir_e dir, func_scrv_cb_t cb)
{
    func_s_tlv_in_t input;
    func_s_params_t params;
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : !(handle->dir_forward);

    if (((handle->scrv.v != 0) && (*(handle->dir_reg_ptr) != b_dir)) || l < 0)
    {
        handle->scrv.error_count++;
        return 0;
    }

    FuncScrv_BufLock(&handle->scrv);
    *(handle->dir_reg_ptr) = b_dir;

    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.j    = handle->scrv.j;
    input.l    = l;
    input.v0   = handle->scrv.v;
    input.vc   = (vc > 0) ? vc : handle->scrv.vmax;
    input.v1   = v1;
    input.t    = 0;

    FuncS_Tlv(&params, &input);
    handle->scrv.buf.sp = params;

    if (params.v1 == 0)
    {
        FuncPlus_CalcTag(handle, dir, l);
    }
    else
    {
        handle->is_fixed_length = 0;
    }

    FuncScrv_BufUnlock(&handle->scrv, 0, cb);
    return 1;
}

/**
  * @brief  按给定初末速度、距离和时间生成 S 曲线运动。
  * @param  handle 步进脉冲通道指针。
  * @param  v0 初始速度。
  * @param  v1 末速度。
  * @param  l 目标运动距离。
  * @param  t 目标运动时间，传入 0 时自动计算最短时间。
  * @param  dir 运动方向。
  * @retval 1 指令写入成功。
  * @retval 0 当前状态不允许写入该运动指令。
  */
unsigned char FuncPlus_STlv(func_plus_ch_t* handle, float v0, float v1,
                               float l, float t, func_plus_dir_e dir)
{
    func_s_params_t params;
    func_s_tlv_in_t input;
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : !(handle->dir_forward);

    if (((handle->scrv.v != 0) && (*(handle->dir_reg_ptr) != b_dir)) || l < 0)
    {
        handle->scrv.error_count++;
        return 0;
    }

    FuncScrv_BufLock(&handle->scrv);
    *(handle->dir_reg_ptr) = b_dir;
    handle->is_fixed_length = 0;

    input.j    = handle->scrv.j;
    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.l    = l;
    input.t    = t;
    input.v0   = v0;
    input.v1   = v1;
    if (input.t == 0)
    {
        input.vc = handle->scrv.vmax;
    }

    FuncS_Tlv(&params, &input);
    handle->scrv.buf.sp = params;

    if ((params.v1 == 0) && (input.l != 0))
    {
        FuncPlus_CalcTag(handle, dir, l);
    }
    else
    {
        handle->is_fixed_length = 0;
    }

    FuncScrv_BufUnlock(&handle->scrv, 0, SCRV_CB1);
    return 1;
}

/**
  * @brief  按 S 曲线平滑切换到目标连续速度。
  * @param  handle 步进脉冲通道指针。
  * @param  v 目标速度。
  * @param  dir 运动方向。
  * @param  cb 启动回调。
  * @retval 1 指令写入成功。
  * @retval 0 当前状态不允许反向切换。
  */
unsigned char FuncPlus_SCv(func_plus_ch_t* handle, float v,
                              func_plus_dir_e dir, func_scrv_cb_t cb)
{
    func_s_params_t params;
    func_s_tlv_in_t input = {0};
    unsigned char b_dir;

    b_dir = (dir == DIR_FORWARD) ? handle->dir_forward : !(handle->dir_forward);

    if ((handle->scrv.v != 0) && (*(handle->dir_reg_ptr) != b_dir))
    {
        handle->scrv.error_count++;
        return 0;
    }

    FuncScrv_BufLock(&handle->scrv);
    *(handle->dir_reg_ptr) = b_dir;

    input.j    = handle->scrv.j;
    input.amax = handle->scrv.amax;
    input.vmax = handle->scrv.vmax;
    input.v0   = handle->scrv.v;
    input.v1   = v;
    input.l    = 0;    /* l=0 时仅进行速度切换，不限制位移。 */
    input.t    = -1;

    FuncS_Tlv(&params, &input);
    handle->scrv.buf.sp = params;
    handle->is_fixed_length = 0;

    FuncScrv_BufUnlock(&handle->scrv, 0, cb);
    return 1;
}

/**
  * @brief  将通道切换为从属速度模式。
  * @param  handle 步进脉冲通道指针。
  * @retval 1 模式切换成功。
  */
unsigned char FuncPlus_SSlv(func_plus_ch_t* handle)
{
    FuncScrv_BufLock(&handle->scrv);
    handle->is_fixed_length = 0;
    FuncScrv_BufUnlock(&handle->scrv, 1, SCRV_CB1);
    return 1;
}

/**
  * @brief  查询通道是否仍处于忙状态。
  * @param  handle 步进脉冲通道指针。
  * @retval 1 通道忙，运动或 S 曲线更新尚未结束。
  * @retval 0 通道空闲。
  */
unsigned char FuncPlus_SBusy(func_plus_ch_t* handle)
{
    unsigned char r;

    if (handle->is_fixed_length)
    {
        r = (handle->scrv.buf.has_new_params || (handle->scrv.step_state != FUNC_SCRV_STD)
             || handle->is_running || (handle->plus_cur != handle->plus_tag));
    }
    else
    {
        r = (handle->scrv.buf.has_new_params || (handle->scrv.step_state != FUNC_SCRV_STD));
    }

    return r;
}

/**
  * @brief  查询 S 曲线指令缓冲区是否为空。
  * @param  handle 步进脉冲通道指针。
  * @retval 1 缓冲区为空，可写入新指令。
  * @retval 0 缓冲区已有待执行指令。
  */
unsigned char FuncPlus_SBufEmpty(func_plus_ch_t* handle)
{
    return (!handle->scrv.buf.has_new_params);
}

/**
  * @brief  清零通道当前脉冲计数。
  * @param  handle 步进脉冲通道指针。
  * @retval 无
  */
void FuncPlus_SClrPlus(func_plus_ch_t* handle)
{
    handle->plus_cur = 0;
}

