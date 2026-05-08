/**
 * @file stepper_app.c
 * @author 19816
 * @brief 步进电机应用层示例源文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#include "stepper_app.h"
#include "stepper_s_curve/func_scrv.h"
#include <stddef.h>
#include <stdio.h>

#define MOTOR_STEPS_PER_REV   4000.0f  /* 步进电机转一圈对应的电机脉冲数。 */
#define MOTOR_LEAD_MM         148.2f    /* 步进电机转一圈的直线位移，单位 mm。 */
#define MOTOR_PULSE_DIST_MM   (MOTOR_LEAD_MM / MOTOR_STEPS_PER_REV) /* 单个脉冲对应的直线位移，单位 mm。 */

#define MOTOR_JERK_MAX        2000.0f  /* j最大加加速度，单位 mm/s^3。 */
#define MOTOR_ACCEL_MAX       200.0f   /* a最大加速度，单位 mm/s^2。 */
#define MOTOR_SPEED_MAX       100.0f   /* v最大直线速度，单位 mm/s。 */

#define MANUAL_TRAVEL_MM      10.0f   /* 101cm手动定长运动距离，单位 mm。 */
#define MANUAL_CRUISE_SPEED   100.0f   /* 手动运动巡航速度，单位 mm/s。 */
#define MANUAL_TLV_TIME_S     2.0f     /* 手动 TLV 曲线目标时间，单位 s。 */

static volatile uint8_t s_stepper_initialized;
static func_plus_t* s_stepper;

/**
  * @brief  初始化步进电机应用层参数和 S 曲线通道。
  * @param  handle 步进脉冲驱动主对象指针。
  * @retval 无
  * @note   需要在进入主循环前调用一次。
  */
void StepperApp_Init(func_plus_t* handle)
{
    func_plus_ch_init_t ch_init = {0};
    func_plus_ch_t* p;

    if (handle == NULL)
    {
        return;
    }

    ch_init.j = MOTOR_JERK_MAX;
    ch_init.amax = MOTOR_ACCEL_MAX;
    ch_init.vmax = MOTOR_SPEED_MAX;
    ch_init.plus_dist = MOTOR_PULSE_DIST_MM;
    ch_init.host_v_ptr = NULL;
    ch_init.dir_reg_ptr = 0; /* 当前未绑定方向引脚，先使用软件方向位。 */
    ch_init.dir_forward = DIR_FORWARD;

    s_stepper = handle;

    FuncPlus_ChInit(&s_stepper->ch[0], &ch_init);
    FuncPlus_Init(s_stepper);
    FuncPlus_SClrPlus(&s_stepper->ch[0]);

    p = &s_stepper->ch[0];

    /*
     * 手动运行选择区：每次只保留一种模式取消注释。
     * StepperApp_Tick1ms() 需由 1ms 调度任务调用，用于推进 S 曲线。
     */

    /* 模式 1：定长运动，按巡航速度走指定距离。 */
    FuncPlus_SL(p, MANUAL_CRUISE_SPEED, 0.0f, MANUAL_TRAVEL_MM, DIR_FORWARD, SCRV_CB1);

    /* 模式 2：按初速、末速、距离、目标时间生成 S 曲线。 */
//	FuncPlus_STlv(p, 0.0f, 0.0f, MANUAL_TRAVEL_MM, MANUAL_TLV_TIME_S, DIR_FORWARD);

    /* 模式 3：连续速度模式，平滑切换到目标速度。 */
// FuncPlus_SCv(p, MANUAL_CRUISE_SPEED, DIR_FORWARD, SCRV_CB1); 

    s_stepper_initialized = 1U;
}

/**
  * @brief  步进电机 1ms 周期节拍处理。
  * @param  无
  * @retval 无
  * @note   初始化完成后调用 FuncPlus_Tick 更新 S 曲线速度。
  */
void StepperApp_Tick1ms(void)
{
    if (s_stepper_initialized != 0U)
    {
        FuncPlus_Tick();
    }
}

/**
  * @brief  获取当前定长运动剩余脉冲数。
  * @param  无
  * @retval uint32_t 当前轴到目标位置的剩余脉冲数。
  * @note   读取脉冲计数时会临时关闭中断，避免中断中更新计数造成竞争。
  */
uint32_t StepperApp_RemainingPulses(void)
{
    func_plus_ch_t* axis;
    uint32_t remaining = 0U;

    if ((s_stepper_initialized == 0U) || (s_stepper == NULL))
    {
        return 0U;
    }

    axis = &s_stepper->ch[0];

    __disable_irq();
    if (axis->is_fixed_length != 0U)
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
  * @brief  查询步进电机应用是否仍在运动。
  * @param  无
  * @retval 1 当前轴忙或 S 曲线状态机未结束。
  * @retval 0 当前轴空闲或应用尚未初始化。
  */
uint8_t StepperApp_IsBusy(void)
{
    if ((s_stepper_initialized == 0U) || (s_stepper == NULL))
    {
        return 0U;
    }

    return FuncPlus_SBusy(&s_stepper->ch[0]);
}

