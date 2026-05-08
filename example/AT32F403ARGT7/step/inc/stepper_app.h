/**
 * @file stepper_app.h
 * @author 19816
 * @brief 步进电机应用接口头文件
 * @version 1.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */
#ifndef STEPPER_APP_H
#define STEPPER_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stepper_app_def.h"

void StepperApp_Init(func_plus_t* handle);
void StepperApp_Tick1ms(void);
uint32_t StepperApp_RemainingPulses(void);
uint8_t StepperApp_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_APP_H */

