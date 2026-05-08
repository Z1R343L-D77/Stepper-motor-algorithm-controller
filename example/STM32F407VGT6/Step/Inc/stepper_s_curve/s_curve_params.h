/**
 * @file s_curve_params.h
 * @author Ws
 * @brief S 曲线参数计算接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef S_CURVE_PARAMS_H
#define S_CURVE_PARAMS_H

#ifdef S_CURVE_PARAMS_MAIN
#define S_CURVE_PARAMS_EXT
#else
#define S_CURVE_PARAMS_EXT  extern
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    S_CURVE_PARAMS_STD = 0,
    S_CURVE_PARAMS_JA1,
    S_CURVE_PARAMS_JA0,
    S_CURVE_PARAMS_JAN1,
    S_CURVE_PARAMS_C,
    S_CURVE_PARAMS_JDN1,
    S_CURVE_PARAMS_JD0,
    S_CURVE_PARAMS_JD1,
    S_CURVE_PARAMS_END
} s_curve_params_step_e;

typedef struct
{
    float l;
    float j;
    float v0;
    float vmax;
    float amax;
} func_s_inputs_t;

typedef struct
{
    float j;
    float vmax;
    float amax;
    float v0;
    float vc;
    float v1;
    float l;
    float t;
} func_s_tlv_in_t;

typedef struct
{
    unsigned char acc_reach_amax;
    unsigned char dec_reach_amax;

    float a_sign;
    float d_sign;

    float vc;
    float t_j_a;
    float t_a_a;
    float t_j_d;
    float t_a_d;
    float t_c;

    float v0;
    float v1;

    float tim;
    float dx_acc;
    float dx_dec;
    float dx_total;
} func_s_params_t;

typedef struct
{
    float t;
    float l;
} func_s_calc_tl_t;

/**
 * @brief 根据输入约束求解 S 曲线参数。
 * @param handle S 曲线参数输出指针。
 * @param input S 曲线输入约束参数指针。
 * @retval 无。
 */
void SolveScurveParams(func_s_params_t *handle, func_s_inputs_t *input);

/**
 * @brief 根据 TLV 约束求解 S 曲线参数。
 * @param handle S 曲线参数输出指针。
 * @param input TLV 输入约束参数指针。
 * @retval 计算结果状态码。
 */
int FuncSTlv(func_s_params_t *handle, func_s_tlv_in_t *input);

/**
 * @brief 计算速度从 v0 变化到 v1 的时间和位移。
 * @param v0 初始速度。
 * @param v1 目标速度。
 * @param amax 最大加速度。
 * @param j 最大加加速度。
 * @retval 时间和位移结果结构体。
 */
func_s_calc_tl_t FuncSCalcTl(float v0, float v1, float amax, float j);

#ifdef __cplusplus
}
#endif

#endif /* S_CURVE_PARAMS_H */

