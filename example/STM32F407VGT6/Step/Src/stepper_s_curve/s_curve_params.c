/**
 * @file s_curve_params.c
 * @author Ws
 * @brief S 曲线运动参数计算实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#include <math.h>

#define S_CURVE_PARAMS_MAIN
#include "s_curve_params.h"

#ifndef S_CURVE_FMAX
#define S_CURVE_FMAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif

#ifndef S_CURVE_FMIN
#define S_CURVE_FMIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif

#define ERR_MIN     (0.0001f)
#define L_CALC_CYC  (32U)
#define T_CALC_CYC  (16U)

typedef struct
{
    float sign;
    unsigned char reached_amax;
    float t_j;
    float t_a;
    float dur;
    float dx;
} mono_segment_params_t;

/**
 * @brief 将输入值限制为不小于指定下限。
 * @param x 输入值。
 * @param mn 最小允许值。
 * @retval 限幅后的结果。
 */
static inline float ClampMin(float x, float mn)
{
    if (x < mn)
    {
        return mn;
    }

    return x;
}

/**
 * @brief 解析单调速度变化段的时间和位移参数。
 * @param handle 单调段计算结果输出指针。
 * @param v_in 初始速度。
 * @param v_out 目标速度。
 * @param amax 最大加速度。
 * @param j 最大加加速度。
 * @retval 无。
 */
static void mono_segment_analytic(mono_segment_params_t *handle, float v_in, float v_out,
                                  float amax, float j)
{
    float sign;
    float dv;
    float dv_tri;

    float tj;
    float ta;
    float v1;
    float v2;
    float dx1;
    float dx2;
    float dx3;

    dv = v_out - v_in;
    if (dv >= 0.0f)
    {
        sign = 1.0f;
    }
    else
    {
        sign = -1.0f;
        dv = -dv;
    }

    handle->sign = sign;

    dv_tri = (amax * amax) / j;

    if (dv < dv_tri)
    {
        handle->reached_amax = 0U;
        handle->t_j = sqrtf(dv / j);
        handle->t_a = 0.0f;
        handle->dur = 2.0f * handle->t_j;

        handle->dx = 2.0f * v_in * handle->t_j +
                     (j * handle->t_j * handle->t_j * handle->t_j) * sign;
    }
    else
    {
        handle->reached_amax = 1U;
        handle->t_j = amax / j;
        handle->t_a = (dv - (amax * amax / j)) / amax;
        if (handle->t_a < 0.0f)
        {
            handle->t_a = 0.0f;
        }
        handle->dur = 2.0f * handle->t_j + handle->t_a;

        tj = handle->t_j;
        ta = handle->t_a;
        v1 = v_in + 0.5f * amax * amax / j * sign;
        v2 = v1 + amax * ta * sign;

        dx1 = v_in * tj + (1.0f / 6.0f) * j * tj * tj * tj * sign;
        dx2 = v1 * ta + 0.5f * amax * ta * ta * sign;
        dx3 = v2 * tj + (0.5f * amax * tj * tj - (1.0f / 6.0f) * j * tj * tj * tj) * sign;

        handle->dx = dx1 + dx2 + dx3;
    }
}

/**
 * @brief 计算无匀速段情况下的总位移。
 * @param v0 初始速度。
 * @param vp 峰值速度。
 * @param amax 最大加速度。
 * @param j 最大加加速度。
 * @retval 加速段和减速段位移之和。
 */
static float s_curve_distance_no_cruise(float v0, float vp,
                                        float amax, float j)
{
    mono_segment_params_t acc;
    mono_segment_params_t dec;

    mono_segment_analytic(&acc, v0, vp, amax, j);
    mono_segment_analytic(&dec, vp, 0.0f, amax, j);
    return acc.dx + dec.dx;
}

/**
 * @brief 根据距离和运动限制求解完整 S 曲线参数。
 * @param handle S 曲线参数输出指针。
 * @param input S 曲线输入约束参数指针。
 * @retval 无。
 */
void SolveScurveParams(func_s_params_t *handle, func_s_inputs_t *input)
{
    float d_accdec_vmax;
    float dx_acc;
    float dx_dec;
    float rem;
    mono_segment_params_t acc;
    mono_segment_params_t dec;
    unsigned int it;

    input->v0 = ClampMin(input->v0, 0.0f);
    input->vmax = ClampMin(input->vmax, 0.0f);
    input->amax = ClampMin(input->amax, 1e-12f);
    input->j = ClampMin(input->j, 1e-12f);

    d_accdec_vmax = s_curve_distance_no_cruise(input->v0, input->vmax, input->amax, input->j);

    handle->a_sign = 1.0f;
    handle->d_sign = -1.0f;

    if (input->l > d_accdec_vmax)
    {
        handle->vc = input->vmax;

        mono_segment_analytic(&acc, input->v0, handle->vc, input->amax, input->j);
        mono_segment_analytic(&dec, handle->vc, 0.0f, input->amax, input->j);

        dx_acc = acc.dx;
        dx_dec = dec.dx;
        rem = input->l - (dx_acc + dx_dec);
        handle->t_c = rem / S_CURVE_FMAX(handle->vc, 1e-12f);
        if (handle->t_c < 0.0f)
        {
            handle->t_c = 0.0f;
        }

        handle->acc_reach_amax = acc.reached_amax;
        handle->dec_reach_amax = dec.reached_amax;

        handle->t_j_a = acc.t_j;
        handle->t_a_a = acc.t_a;
        handle->t_j_d = dec.t_j;
        handle->t_a_d = dec.t_a;
        handle->dx_acc = dx_acc;
        handle->dx_dec = dx_dec;
        handle->dx_total = dx_acc + handle->vc * handle->t_c + dx_dec;
        handle->tim = acc.dur + handle->t_c + dec.dur;
    }
    else
    {
        float lo = S_CURVE_FMAX(input->v0, 1e-12f);
        float hi = S_CURVE_FMAX(lo, input->vmax);

        for (it = 0U; it < 80U; ++it)
        {
            float mid = 0.5f * (lo + hi);
            float dmid = s_curve_distance_no_cruise(input->v0, mid, input->amax, input->j);

            if (dmid < input->l)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }

            if (fabsf(hi - lo) <= 1e-12f * S_CURVE_FMAX(1.0f, mid))
            {
                break;
            }
        }

        handle->vc = 0.5f * (lo + hi);
        handle->t_c = 0.0f;

        mono_segment_analytic(&acc, input->v0, handle->vc, input->amax, input->j);
        mono_segment_analytic(&dec, handle->vc, 0.0f, input->amax, input->j);

        handle->acc_reach_amax = acc.reached_amax;
        handle->dec_reach_amax = dec.reached_amax;
        handle->t_j_a = acc.t_j;
        handle->t_a_a = acc.t_a;
        handle->t_j_d = dec.t_j;
        handle->t_a_d = dec.t_a;

        handle->dx_acc = acc.dx;
        handle->dx_dec = dec.dx;
        handle->dx_total = acc.dx + dec.dx;
        handle->tim = acc.dur + dec.dur;
    }
}

#define S_TLV_SUCCESS  (1)
#define S_TLV_T_SHORT  (-1)
#define S_TLV_V02V1    (2)
#define S_TLV_L_VC     (3)
#define S_TLV_L_NVC    (4)
#define S_TLV_L_SHORT  (-2)

/**
 * @brief 根据初末速度、距离和时间约束计算 S 曲线参数。
 * @param handle S 曲线参数输出指针。
 * @param input TLV 输入约束参数指针。
 * @retval 计算状态码。
 */
int FuncSTlv(func_s_params_t *handle, func_s_tlv_in_t *input)
{
    float l;
    float vc;
    float dl;
    float hi;
    float lo;
    float f;
    float tc;
    int r;

    unsigned int it;

    mono_segment_params_t acc;
    mono_segment_params_t dec = {0};

    r = 0;
    vc = input->vc;
    tc = 0.0f;

    if (input->v0 > input->vmax)
    {
        input->v0 = input->vmax;
    }
    if (input->v1 > input->vmax)
    {
        input->v1 = input->vmax;
    }
    if (input->vc > input->vmax)
    {
        input->vc = input->vmax;
    }

    while (r == 0)
    {
        if (input->t == 0.0f)
        {
            vc = input->vc;
            mono_segment_analytic(&acc, input->v0, vc, input->amax, input->j);
            mono_segment_analytic(&dec, vc, input->v1, input->amax, input->j);

            if ((acc.dx + dec.dx) <= input->l)
            {
                f = input->l - acc.dx - dec.dx;
                f /= vc;
                tc = f;
                r = S_TLV_L_VC;
                break;
            }
            else
            {
                hi = vc;
                lo = 0.0f;
                l = input->l;
                for (it = 0U; it < L_CALC_CYC; ++it)
                {
                    vc = (hi + lo) / 2.0f;

                    mono_segment_analytic(&acc, input->v0, vc, input->amax, input->j);
                    mono_segment_analytic(&dec, vc, input->v1, input->amax, input->j);

                    dl = (acc.dx + dec.dx) - l;

                    if (fabsf(dl) < ERR_MIN)
                    {
                        break;
                    }

                    if (dl > 0.0f)
                    {
                        hi = vc;
                    }
                    else
                    {
                        lo = vc;
                    }
                }

                if (it == L_CALC_CYC)
                {
                    r = S_TLV_L_SHORT;
                }
                else
                {
                    r = S_TLV_L_NVC;
                }
                break;
            }
        }

        mono_segment_analytic(&acc, input->v0, input->v1, input->amax, input->j);
        if ((acc.dur > input->t) && (input->t != 0.0f))
        {
            r = S_TLV_T_SHORT;
            break;
        }

        if (input->l == 0.0f)
        {
            r = S_TLV_V02V1;
        }

        hi = input->vmax;
        lo = 0.0f;
        l = input->l;
        tc = 0.0f;
        for (it = 0U; it < L_CALC_CYC; ++it)
        {
            vc = (hi + lo) / 2.0f;

            mono_segment_analytic(&acc, input->v0, vc, input->amax, input->j);
            mono_segment_analytic(&dec, vc, input->v1, input->amax, input->j);

            if (((acc.dx + dec.dx) > l) || ((acc.dur + dec.dur) > input->t))
            {
                hi = vc;
            }
            else
            {
                tc = input->t - (acc.dur + dec.dur);
                dl = (acc.dx + dec.dx + tc * vc) - l;

                if (fabsf(dl) <= ERR_MIN)
                {
                    handle->vc = vc;
                    break;
                }
                else if (dl < 0.0f)
                {
                    lo = vc;
                }
                else
                {
                    hi = vc;
                }
            }
        }

        if (it != L_CALC_CYC)
        {
            r = S_TLV_SUCCESS;
        }
        else
        {
            hi = input->vmax;
            lo = 0.0f;
            l = input->l;
            for (it = 0U; it < T_CALC_CYC; ++it)
            {
                vc = (hi + lo) / 2.0f;
                mono_segment_analytic(&acc, input->v0, vc, input->amax, input->j);
                mono_segment_analytic(&dec, vc, input->v1, input->amax, input->j);

                if ((acc.dx + dec.dx) > l)
                {
                    hi = vc;
                }
                else
                {
                    dl = l - (acc.dx + dec.dx);
                    tc = dl / vc;

                    if ((input->t - (acc.dur + dec.dur + tc)) < 0.0f)
                    {
                        lo = vc;
                    }
                    else
                    {
                        hi = vc;
                    }
                }
            }
            r = S_TLV_T_SHORT;
            break;
        }
    }

    handle->v0 = input->v0;
    handle->v1 = input->v1;
    handle->a_sign = acc.sign;
    handle->t_j_a = acc.t_j;
    handle->t_a_a = acc.t_a;
    handle->t_c = tc;
    handle->vc = vc;
    handle->d_sign = dec.sign;
    handle->t_j_d = dec.t_j;
    handle->t_a_d = dec.t_a;
    handle->dx_acc = acc.dx;
    handle->dx_dec = dec.dx;
    handle->dx_total = acc.dx + dec.dx + tc * vc;
    handle->tim = handle->t_j_a * 2.0f + handle->t_a_a + handle->t_c + handle->t_j_d * 2.0f + handle->t_a_d;
    return r;
}

/**
 * @brief 计算速度从 v0 变化到 v1 所需时间和距离。
 * @param v0 初始速度。
 * @param v1 目标速度。
 * @param amax 最大加速度。
 * @param j 最大加加速度。
 * @retval 速度变化所需时间和位移。
 */
func_s_calc_tl_t FuncSCalcTl(float v0, float v1, float amax, float j)
{
    mono_segment_params_t acc;
    func_s_calc_tl_t tl;

    mono_segment_analytic(&acc, v0, v1, amax, j);
    tl.l = acc.dx;
    tl.t = acc.dur;
    return tl;
}

