/**
 * @file func_scrv.c
 * @author 19816
 * @brief S 曲线状态机源文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#define FUNC_SCRV_MAIN
#include "func_scrv.h"
#include <stdio.h>
#define FUNC_SCRV_T_UNI_ADD (FUNC_SCRV_T_UNI * 0.99f)

/**
  * @brief  S 曲线默认等待回调。
  * @param  无
  * @retval 0 表示暂不启动新曲线。
  */
unsigned char FuncScrv_Cb0(void)
{
    return 0;
}

/**
  * @brief  S 曲线默认立即执行回调。
  * @param  无
  * @retval 1 表示允许启动新曲线。
  */
unsigned char FuncScrv_Cb1(void)
{
    return 1;
}

/**
  * @brief  执行一次 S 曲线速度状态机计算。
  * @param  handle S 曲线状态机对象指针。
  * @retval float 当前 tick 计算得到的目标速度。
  * @note   需要按 FUNC_SCRV_T_UNI 周期调用。
  */
float FuncScrv_Tick(func_scrv_t* handle)
{
    float dt;
    func_s_params_t* params = &handle->sp;
    unsigned char b_start = 0;

    if (!handle->buf.is_locked && handle->buf.has_new_params && (handle->buf.cb)() == 1)
    {
        handle->buf.has_new_params = 0;
        handle->step_state = FUNC_SCRV_STD;
        handle->is_slave = handle->buf.is_slave;
        b_start = 1;
    }

    switch (handle->step_state)
    {
    case FUNC_SCRV_STD:
        if (!handle->is_slave)
        {
            if (b_start)
            {
                handle->sp = handle->buf.sp;
                handle->t = FUNC_SCRV_T_UNI;
                handle->a = 0;
                handle->t_last = 0;
                handle->a_last = 0;
                handle->v_last = params->v0;
                handle->tx = params->t_ja;
                handle->step_state = FUNC_SCRV_JA1;
            }
            else
            {
                break;
            }
        }
        else
        {
            return *handle->host_v_ptr;
        }

    case FUNC_SCRV_JA1:
        if (handle->t >= handle->tx)
        {
            dt = params->t_ja;
            handle->a_last = (handle->j * params->a_sign) * dt;
            handle->v_last += (handle->j * params->a_sign) * dt * dt / 2;
            handle->t_last += dt;
            handle->tx += params->t_aa;
            handle->step_state = FUNC_SCRV_JA0;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->a = (handle->j * params->a_sign) * dt;
            handle->v = handle->v_last + ((handle->j * params->a_sign) * dt * dt / 2);
            break;
        }

    case FUNC_SCRV_JA0:
        if (handle->t >= handle->tx)
        {
            dt = params->t_aa;
            handle->t_last += dt;
            handle->v_last += handle->a_last * dt;
            handle->tx += params->t_ja;
            handle->step_state = FUNC_SCRV_JAN1;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->v = handle->v_last + handle->a_last * dt;
            break;
        }

    case FUNC_SCRV_JAN1:
        if (handle->t >= handle->tx)
        {
            dt = params->t_ja;
            handle->a_last = handle->a_last - (handle->j * params->a_sign) * dt;
            handle->v_last = handle->v_last + (handle->j * params->a_sign) * dt * dt / 2;
            handle->t_last += dt;
            handle->a = 0;
            handle->tx += params->t_c;
            handle->step_state = FUNC_SCRV_C;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->a = handle->a_last - (handle->j * params->a_sign) * dt;
            dt = (2 * params->t_ja - dt) * dt;
            handle->v = handle->v_last + (handle->j * params->a_sign) * dt / 2;
            break;
        }

    case FUNC_SCRV_C:
        if (handle->t >= handle->tx)
        {
            dt = params->t_c;
            handle->t_last += dt;
            handle->a_last = 0;
            handle->tx += params->t_jd;
            handle->step_state = FUNC_SCRV_JDN1;
        }
        else
        {
            handle->v = handle->v_last;
            break;
        }

    case FUNC_SCRV_JDN1:
        if (handle->t >= handle->tx)
        {
            dt = params->t_jd;
            handle->a_last += (handle->j * params->d_sign) * dt;
            handle->v_last += (handle->j * params->d_sign) * dt * dt / 2;
            handle->t_last += dt;
            handle->tx += params->t_ad;
            handle->step_state = FUNC_SCRV_JD0;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->a = handle->a_last + (handle->j * params->d_sign) * dt;
            handle->v = handle->v_last + (handle->j * params->d_sign) * dt * dt / 2;
            break;
        }

    case FUNC_SCRV_JD0:
        if (handle->t >= handle->tx)
        {
            dt = params->t_ad;
            handle->v_last += handle->a_last * dt;
            handle->t_last += dt;
            handle->tx += params->t_jd;
            handle->step_state = FUNC_SCRV_JD1;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->a = handle->a_last;
            handle->v = handle->v_last + handle->a * dt;
            break;
        }

    case FUNC_SCRV_JD1:
        if (handle->t >= handle->tx)
        {
            dt = params->t_jd;
            handle->a_last -= (handle->j * params->d_sign) * dt;
            handle->v_last += (handle->j * params->d_sign) * dt * dt / 2;
            handle->a = 0;
            handle->v = params->v1;
            handle->step_state = FUNC_SCRV_STD;
        }
        else
        {
            dt = handle->t - handle->t_last;
            handle->a = handle->a_last - (handle->j * params->d_sign) * dt;
            dt = (2 * params->t_jd - dt) * dt;
            handle->v = handle->v_last + (handle->j * params->d_sign) * dt / 2;
        }
        break;

    case FUNC_SCRV_END:
        break;
    }

    handle->t += FUNC_SCRV_T_UNI;
    return handle->v;
}

/**
  * @brief  查询 S 曲线参数缓冲区是否为空。
  * @param  handle S 曲线状态机对象指针。
  * @retval 1 缓冲区为空。
  * @retval 0 缓冲区存在待执行参数。
  */
unsigned char FuncScrv_BufIsEmpty(func_scrv_t* handle)
{
    return !handle->buf.has_new_params;
}

/**
  * @brief  锁定 S 曲线参数缓冲区。
  * @param  handle S 曲线状态机对象指针。
  * @retval 无
  * @note   写入新曲线参数前调用，避免状态机读取半更新数据。
  */
void FuncScrv_BufLock(func_scrv_t* handle)
{
    handle->buf.is_locked = 1;
}

/**
  * @brief  解锁 S 曲线参数缓冲区并提交新参数。
  * @param  handle S 曲线状态机对象指针。
  * @param  is_slave 是否进入从属速度模式。
  * @param  cb 新曲线启动条件回调。
  * @retval 无
  */
void FuncScrv_BufUnlock(func_scrv_t* handle, unsigned char is_slave, func_scrv_cb_t cb)
{
    func_scrv_buf_t* buffer = &handle->buf;

    if (cb == NULL)
    {
        buffer->cb = &FuncScrv_Cb0;
    }
    else if ((unsigned int)cb == 1U)
    {
        buffer->cb = &FuncScrv_Cb1;
    }
    else
    {
        buffer->cb = cb;
    }

    buffer->is_slave = is_slave;
    buffer->has_new_params = 1;
    buffer->is_locked = 0;
}

