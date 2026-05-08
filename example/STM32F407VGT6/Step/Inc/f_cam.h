/**
 * @file f_cam.h
 * @author Ws
 * @brief 步进电机应用接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef STEPPER_APP_H
#define STEPPER_APP_H

#include <stdint.h>

#include "f_cam_def.h"

#define Stepper_AppInit             StepperAppInit
#define Stepper_AppTick1ms          StepperAppTick1ms
#define stepper_app_tick_1ms  StepperAppTick1ms
#define Stepper_AppRemainingPulses  StepperAppRemainingPulses
#define Stepper_AppIsBusy           StepperAppIsBusy
#define Stepper_AppMoveMm           StepperAppMoveMm
#define Stepper_AppSetSpeedMmS      StepperAppSetSpeedMmS
#define Stepper_AppStop             StepperAppStop
#define Stepper_AppZeroPosition     StepperAppZeroPosition
#define Stepper_AppPositionMm       StepperAppPositionMm

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化步进电机应用层。
 * @param handle 步进脉冲驱动主对象指针。
 * @retval 无。
 */
void StepperAppInit(func_plus_t *handle);

/**
 * @brief 步进电机应用 1ms 周期节拍。
 * @param 无。
 * @retval 无。
 */
void StepperAppTick1ms(void);

/**
 * @brief 获取剩余脉冲数。
 * @param 无。
 * @retval 当前剩余脉冲数。
 */
uint32_t StepperAppRemainingPulses(void);

/**
 * @brief 查询步进电机是否忙。
 * @param 无。
 * @retval 1 忙。
 * @retval 0 空闲。
 */
uint8_t StepperAppIsBusy(void);

/**
 * @brief 发起有符号相对位移运动。
 * @param distance_mm 有符号位移距离，正数为正转，负数为反转。
 * @param speed_mm_s 巡航速度，单位 mm/s。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */
uint8_t StepperAppMoveMm(float distance_mm, float speed_mm_s);

/**
 * @brief 切换为有符号连续速度模式。
 * @param speed_mm_s 有符号目标速度，单位 mm/s。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */
uint8_t StepperAppSetSpeedMmS(float speed_mm_s);

/**
 * @brief 请求平滑停止。
 * @param 无。
 * @retval 1 指令受理成功。
 * @retval 0 指令被拒绝。
 */
uint8_t StepperAppStop(void);

/**
 * @brief 清零当前位置脉冲计数。
 * @param 无。
 * @retval 无。
 */
void StepperAppZeroPosition(void);

/**
 * @brief 获取当前位置，单位毫米。
 * @param 无。
 * @retval 当前位移，单位 mm。
 */
float StepperAppPositionMm(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_APP_H */

