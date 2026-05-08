/**
 * @file func_scrv.h
 * @author 19816
 * @brief S 曲线状态机接口头文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#ifndef FUNC_SCRV_H
#define FUNC_SCRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "s_curve_params.h"

#ifdef FUNC_SCRV_MAIN
#define FUNC_SCRV_EXT
#else
#define FUNC_SCRV_EXT extern
#endif

#define FUNC_SCRV_T_UNI 0.001f
#define SCRV_CB0        (&FuncScrv_Cb0)
#define SCRV_CB1        (&FuncScrv_Cb1)

enum
{
    FUNC_SCRV_MODE_S,
    FUNC_SCRV_MODE_SLV
};

enum
{
    FUNC_SCRV_STD,
    FUNC_SCRV_JA1,
    FUNC_SCRV_JA0,
    FUNC_SCRV_JAN1,
    FUNC_SCRV_C,
    FUNC_SCRV_JDN1,
    FUNC_SCRV_JD0,
    FUNC_SCRV_JD1,
    FUNC_SCRV_END
};

typedef unsigned char (*func_scrv_cb_t)(void);

typedef struct
{
    unsigned char is_locked;
    unsigned char has_new_params;
    unsigned char is_slave;
    func_scrv_cb_t cb;
    func_s_params_t sp;
} func_scrv_buf_t;

typedef struct
{
    func_scrv_buf_t buf;
    unsigned char step_state;
    unsigned char error_count;
    unsigned char is_slave;
    float *host_v_ptr;
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

float FuncScrv_Tick(func_scrv_t* handle);
unsigned char FuncScrv_Cb0(void);
unsigned char FuncScrv_Cb1(void);
unsigned char FuncScrv_BufIsEmpty(func_scrv_t* handle);
void FuncScrv_BufLock(func_scrv_t* handle);
void FuncScrv_BufUnlock(func_scrv_t* handle, unsigned char is_slave, func_scrv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* FUNC_SCRV_H */

