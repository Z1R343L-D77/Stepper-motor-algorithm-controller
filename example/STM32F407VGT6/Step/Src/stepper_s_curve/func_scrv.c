/**
 * @file func_scrv.c
 * @author Ws
 * @brief S 曲线速度状态机实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#define FUNC_SCRV_MAIN
#include "func_scrv.h"

#define FUNC_SCRV_T_UNI_ADD  (FUNC_SCRV_T_UNI * 0.99f)

/**
 * @brief S 曲线默认等待回调。
 * @param 无。
 * @retval 0 表示暂不启动新曲线。
 */
unsigned char FuncScrvCb0(void)
{
    return 0U;
}

/**
 * @brief S 曲线默认立即执行回调。
 * @param 无。
 * @retval 1 表示允许启动新曲线。
 */
unsigned char FuncScrvCb1(void)
{
    return 1U;
}

/**
 * @brief 执行一次 S 曲线速度状态机计算。
 * @param handle S 曲线状态机对象指针。
 * @retval 当前 tick 计算得到的目标速度。
 */
float FuncScrvTick(func_scrv_t *handle)
{
    float dt;
    func_s_params_t *params;
    unsigned char b_start = 0U;

    params = &handle->sp;

    if ((handle->buf.b_lock == 0U) && (handle->buf.b_new_sp != 0U) && ((handle->buf.cb)() == 1U))
    {
        handle->buf.b_new_sp = 0U;
        handle->step = FUNC_SCRV_STD;
        handle->b_slv = handle->buf.b_slv;
        b_start = 1U;
    }

    switch (handle->step)
    {
        case FUNC_SCRV_STD:
            if (handle->b_slv == 0U)
            {
                if (b_start != 0U)
                {
                    handle->sp = handle->buf.sp;
                    handle->t = FUNC_SCRV_T_UNI;
                    handle->a = 0.0f;
                    handle->t_last = 0.0f;
                    handle->a_last = 0.0f;
                    handle->v_last = params->v0;
                    handle->tx = params->t_j_a;
                    handle->step = FUNC_SCRV_JA1;
                }
                else
                {
                    break;
                }
            }
            else
            {
                return *(handle->phost_v);
            }

        case FUNC_SCRV_JA1:
            if (handle->t >= handle->tx)
            {
                dt = params->t_j_a;
                handle->a_last = (handle->j * params->a_sign) * dt;
                handle->v_last += (handle->j * params->a_sign) * dt * dt / 2.0f;
                handle->t_last += dt;
                handle->tx += params->t_a_a;
                handle->step = FUNC_SCRV_JA0;
            }
            else
            {
                dt = handle->t - handle->t_last;
                handle->a = (handle->j * params->a_sign) * dt;
                handle->v = handle->v_last + ((handle->j * params->a_sign) * dt * dt / 2.0f);
                break;
            }

        case FUNC_SCRV_JA0:
            if (handle->t >= handle->tx)
            {
                dt = params->t_a_a;
                handle->t_last += dt;
                handle->v_last += handle->a_last * dt;
                handle->tx += params->t_j_a;
                handle->step = FUNC_SCRV_JAN1;
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
                dt = params->t_j_a;
                handle->a_last = handle->a_last - (handle->j * params->a_sign) * dt;
                handle->v_last = handle->v_last + (handle->j * params->a_sign) * dt * dt / 2.0f;
                handle->t_last += dt;
                handle->a = 0.0f;
                handle->tx += params->t_c;
                handle->step = FUNC_SCRV_C;
            }
            else
            {
                dt = handle->t - handle->t_last;
                handle->a = handle->a_last - (handle->j * params->a_sign) * dt;
                dt = (2.0f * params->t_j_a - dt) * dt;
                handle->v = handle->v_last + (handle->j * params->a_sign) * dt / 2.0f;
                break;
            }

        case FUNC_SCRV_C:
            if (handle->t >= handle->tx)
            {
                dt = params->t_c;
                handle->t_last += dt;
                handle->a_last = 0.0f;
                handle->tx += params->t_j_d;
                handle->step = FUNC_SCRV_JDN1;
            }
            else
            {
                handle->v = handle->v_last;
                break;
            }

        case FUNC_SCRV_JDN1:
            if (handle->t >= handle->tx)
            {
                dt = params->t_j_d;
                handle->a_last += (handle->j * params->d_sign) * dt;
                handle->v_last += (handle->j * params->d_sign) * dt * dt / 2.0f;
                handle->t_last += dt;
                handle->tx += params->t_a_d;
                handle->step = FUNC_SCRV_JD0;
            }
            else
            {
                dt = handle->t - handle->t_last;
                handle->a = handle->a_last + (handle->j * params->d_sign) * dt;
                handle->v = handle->v_last + (handle->j * params->d_sign) * dt * dt / 2.0f;
                break;
            }

        case FUNC_SCRV_JD0:
            if (handle->t >= handle->tx)
            {
                dt = params->t_a_d;
                handle->v_last += handle->a_last * dt;
                handle->t_last += dt;
                handle->tx += params->t_j_d;
                handle->step = FUNC_SCRV_JD1;
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
                dt = params->t_j_d;
                handle->a_last -= (handle->j * params->d_sign) * dt;
                handle->v_last += (handle->j * params->d_sign) * dt * dt / 2.0f;
                handle->a = 0.0f;
                handle->v = params->v1;
                handle->step = FUNC_SCRV_STD;
            }
            else
            {
                dt = handle->t - handle->t_last;
                handle->a = handle->a_last - (handle->j * params->d_sign) * dt;
                dt = (2.0f * params->t_j_d - dt) * dt;
                handle->v = handle->v_last + (handle->j * params->d_sign) * dt / 2.0f;
            }
            break;

        case FUNC_SCRV_END:
            break;

        default:
            break;
    }

    handle->t += FUNC_SCRV_T_UNI;
    return handle->v;
}

/**
 * @brief 查询 S 曲线参数缓冲区是否为空。
 * @param handle S 曲线状态机对象指针。
 * @retval 1 缓冲区为空。
 * @retval 0 缓冲区存在待执行参数。
 */
unsigned char FuncScrvBufIsEmpty(func_scrv_t *handle)
{
    return (unsigned char)(handle->buf.b_new_sp == 0U);
}

/**
 * @brief 锁定 S 曲线参数缓冲区。
 * @param handle S 曲线状态机对象指针。
 * @retval 无。
 */
void FuncScrvBufLock(func_scrv_t *handle)
{
    handle->buf.b_lock = 1U;
}

/**
 * @brief 解锁 S 曲线参数缓冲区并提交新参数。
 * @param handle S 曲线状态机对象指针。
 * @param b_slv 是否进入从属速度模式。
 * @param cb 新曲线启动条件回调。
 * @retval 无。
 */
void FuncScrvBufUnlock(func_scrv_t *handle, unsigned char b_slv, func_scrv_cb_t cb)
{
    func_scrv_buf_t *buffer = &handle->buf;

    if (cb == 0)
    {
        buffer->cb = &FuncScrvCb0;
    }
    else if ((unsigned int)cb == 1U)
    {
        buffer->cb = &FuncScrvCb1;
    }
    else
    {
        buffer->cb = cb;
    }

    buffer->b_slv = b_slv;
    buffer->b_new_sp = 1U;
    buffer->b_lock = 0U;
}

