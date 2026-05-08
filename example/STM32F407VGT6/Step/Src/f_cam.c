/**
 * @file f_cam.c
 * @author Ws
 * @brief 步进电机应用层实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "f_cam.h"
#include "main.h"
#include "stepper_s_curve/func_scrv.h"
#include <stdio.h>

#define MOTOR_STEPS_PER_REV  (800.0f)
#define MOTOR_LEAD_MM        (100.0f)
#define MOTOR_PULSE_DIST_MM  (MOTOR_LEAD_MM / MOTOR_STEPS_PER_REV)

#define MOTOR_JERK_MAX       (2000.0f)
#define MOTOR_ACCEL_MAX      (200.0f)
#define MOTOR_SPEED_MAX      (100.0f)

#define MANUAL_TRAVEL_MM     (100.0f)
#define MANUAL_CRUISE_SPEED  (200.0f)
#define MANUAL_TLV_TIME_S    (2.0f)
#define MOTOR_SPEED_MIN      (1.0f)

static volatile uint8_t s_stepper_initialized;
static func_plus_t *s_stepper;

/**
 * @brief 获取默认控制轴对象指针。
 * @param 无。
 * @retval 控制轴对象指针，未初始化时返回 NULL。
 */

static func_plus_ch_t *StepperAppAxis(void)
{
    if ((s_stepper_initialized == 0U) || (s_stepper == NULL))
    {
        return NULL;
    }

    return &s_stepper->ch[0];
}

/**
 * @brief 计算浮点数绝对值。
 * @param value 输入值。
 * @retval 输入值绝对值。
 */

static float StepperAppAbsf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

/**
 * @brief 对目标速度做上下限约束。
 * @param speed_mm_s 输入速度，单位 mm/s。
 * @retval 限幅后的速度值。
 */

static float StepperAppLimitSpeed(float speed_mm_s)
{
    speed_mm_s = StepperAppAbsf(speed_mm_s);

    if (speed_mm_s < MOTOR_SPEED_MIN)
    {
        speed_mm_s = MOTOR_SPEED_MIN;
    }
    else if (speed_mm_s > MOTOR_SPEED_MAX)
    {
        speed_mm_s = MOTOR_SPEED_MAX;
    }
    else
    {
        /* 速度在范围内时保持原值。 */

    }

    return speed_mm_s;
}

/**
 * @brief 读取当前轴方向。
 * @param handle 控制轴对象指针。
 * @retval DIR_FORWARD 正向。
 * @retval DIR_INVERSE 反向。
 */

static func_plus_dir_e StepperAppCurrentDir(func_plus_ch_t *handle)
{
    if (*(handle->p_dir) == handle->dir_forward)
    {
        return DIR_FORWARD;
    }

    return DIR_INVERSE;
}

/**
 * @brief 初始化步进电机应用层参数和 S 曲线通道。
 * @param handle 步进脉冲驱动主对象指针。
 * @retval 无。
 */

void StepperAppInit(func_plus_t *handle)
{
    func_plus_ch_init_t ch_init = {0};

    if (handle == NULL)
    {
        return;
    }

    ch_init.j = MOTOR_JERK_MAX;
    ch_init.amax = MOTOR_ACCEL_MAX;
    ch_init.vmax = MOTOR_SPEED_MAX;
    ch_init.plus_dist = MOTOR_PULSE_DIST_MM;
    ch_init.phost_v = NULL;
    ch_init.p_dir = 0;
    ch_init.dir_forward = DIR_FORWARD;

    s_stepper = handle;

    FuncPlusChInit(&s_stepper->ch[0], &ch_init);
    FuncPlusInit(s_stepper);
    FuncPlusSClrPlus(&s_stepper->ch[0]);

    /* 手动运行选择区：每次只保留一种模式取消注释。 */

    FuncPlusSL(&s_stepper->ch[0], MANUAL_CRUISE_SPEED, 0.0f, MANUAL_TRAVEL_MM, DIR_FORWARD, SCRV_CB1); 

    /* FuncPlusSTlv(&s_stepper->ch[0], 0.0f, 0.0f, MANUAL_TRAVEL_MM, MANUAL_TLV_TIME_S, DIR_FORWARD); */

    /* FuncPlusSCv(&s_stepper->ch[0], MANUAL_CRUISE_SPEED, DIR_FORWARD, SCRV_CB1); */


    s_stepper_initialized = 1U;
}

/**
 * @brief 步进电机 1ms 周期节拍处理。
 * @param 无。
 * @retval 无。
 */

void StepperAppTick1ms(void)
{
    if (s_stepper_initialized != 0U)
    {
        FuncPlusTick();
    }
}

/**
 * @brief 获取当前定长运动剩余脉冲数。
 * @param 无。
 * @retval 当前轴到目标位置的剩余脉冲数。
 */

uint32_t StepperAppRemainingPulses(void)
{
    func_plus_ch_t *axis;
    uint32_t remaining = 0U;

    if ((s_stepper_initialized == 0U) || (s_stepper == NULL))
    {
        return 0U;
    }

    axis = &s_stepper->ch[0];

    __disable_irq();
    if (axis->b_fl != 0U)
    {
        if (axis->plus_tag >= axis->plus_cur)
        {
            remaining = axis->plus_tag - axis->plus_cur;
        }
        else
        {
            remaining = axis->plus_cur - axis->plus_tag;
        }
    }
    __enable_irq();

    return remaining;
}

/**
 * @brief 查询步进电机应用是否仍在运动。
 * @param 无。
 * @retval 1 当前轴忙或 S 曲线状态机未结束。
 * @retval 0 当前轴空闲或应用尚未初始化。
 */

uint8_t StepperAppIsBusy(void)
{
    if ((s_stepper_initialized == 0U) || (s_stepper == NULL))
    {
        return 0U;
    }

    return FuncPlusSBusy(&s_stepper->ch[0]);
}

/**
 * @brief 发起有符号相对位移运动。
 * @param distance_mm 有符号位移距离。
 * @param speed_mm_s 巡航速度，单位 mm/s。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */

uint8_t StepperAppMoveMm(float distance_mm, float speed_mm_s)
{
    func_plus_ch_t *axis = StepperAppAxis();
    func_plus_dir_e dir;
    float distance_abs;

    if (axis == NULL)
    {
        return 0U;
    }

    if ((distance_mm == 0.0f) || (StepperAppIsBusy() != 0U))
    {
        return 0U;
    }

    if (distance_mm >= 0.0f)
    {
        dir = DIR_FORWARD;
    }
    else
    {
        dir = DIR_INVERSE;
    }

    distance_abs = StepperAppAbsf(distance_mm);

    return FuncPlusSL(axis, StepperAppLimitSpeed(speed_mm_s),
                         0.0f, distance_abs, dir, SCRV_CB1);
}

/**
 * @brief 切换为有符号连续速度模式。
 * @param speed_mm_s 有符号目标速度，单位 mm/s。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */

uint8_t StepperAppSetSpeedMmS(float speed_mm_s)
{
    func_plus_ch_t *axis = StepperAppAxis();
    func_plus_dir_e dir;

    if (axis == NULL)
    {
        return 0U;
    }

    if (speed_mm_s == 0.0f)
    {
        return StepperAppStop();
    }

    if (speed_mm_s >= 0.0f)
    {
        dir = DIR_FORWARD;
    }
    else
    {
        dir = DIR_INVERSE;
    }

    return FuncPlusSCv(axis, StepperAppLimitSpeed(speed_mm_s), dir, SCRV_CB1);
}

/**
 * @brief 请求平滑停止。
 * @param 无。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */

uint8_t StepperAppStop(void)
{
    func_plus_ch_t *axis = StepperAppAxis();

    if (axis == NULL)
    {
        return 0U;
    }

    return FuncPlusSCv(axis, 0.0f, StepperAppCurrentDir(axis), SCRV_CB1);
}

/**
 * @brief 清零当前位置脉冲计数。
 * @param 无。
 * @retval 无。
 */

void StepperAppZeroPosition(void)
{
    func_plus_ch_t *axis = StepperAppAxis();

    if (axis == NULL)
    {
        return;
    }

    __disable_irq();
    axis->plus_cur = 0U;
    axis->plus_tag = 0U;
    __enable_irq();
}

/**
 * @brief 获取当前位置，单位毫米。
 * @param 无。
 * @retval 当前位移，单位 mm。
 */

float StepperAppPositionMm(void)
{
    func_plus_ch_t *axis = StepperAppAxis();
    int32_t pulses;

    if (axis == NULL)
    {
        return 0.0f;
    }

    __disable_irq();
    pulses = (int32_t)axis->plus_cur;
    __enable_irq();

    return ((float)pulses) * MOTOR_PULSE_DIST_MM;
}

