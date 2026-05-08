/**
 * @file s_curve_params.h
 * @author 19816
 * @brief S 曲线参数计算接口头文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#ifndef S_CURVE_PARAMS_H
#define S_CURVE_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef S_CURVE_PARAMS_MAIN
#define S_CURVE_PARAMS_EXT
#else
#define S_CURVE_PARAMS_EXT extern
#endif

enum
{
    S_CURVE_PARAMS_STD,
    S_CURVE_PARAMS_JA1,
    S_CURVE_PARAMS_JA0,
    S_CURVE_PARAMS_JAN1,
    S_CURVE_PARAMS_C,
    S_CURVE_PARAMS_JDN1,
    S_CURVE_PARAMS_JD0,
    S_CURVE_PARAMS_JD1,
    S_CURVE_PARAMS_END
};

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
    float t_ja;
    float t_aa;
    float t_jd;
    float t_ad;
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

void SolveScurveParams(func_s_params_t* handle, func_s_inputs_t* input);
int FuncS_Tlv(func_s_params_t* handle, func_s_tlv_in_t* input);
func_s_calc_tl_t FuncS_CalcTl(float v0, float v1, float amax, float j);

#ifdef __cplusplus
}
#endif

#endif /* S_CURVE_PARAMS_H */

