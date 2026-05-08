/**
 * @file drv_gpio.h
 * @author Ws
 * @brief GPIO 控制驱动对外接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_002_G_MYGPIO_H
#define INC_002_G_MYGPIO_H

#include "config.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    GPIO_LOW = 0U,
    GPIO_HIGH
} gpio_level_e;

typedef struct
{
    GPIO_TypeDef *port;
    uint32_t pin;
    uint32_t period;
    uint16_t times;
    uint16_t cnt;
    uint16_t times_cnt;
    uint8_t reset;
    uint8_t end;
    float light_on_percent;
    gpio_level_e level;
} gpio_control_t;

/**
 * @brief 初始化 GPIO 闪烁控制对象。
 * @param handle GPIO 控制对象指针。
 * @retval 无。
 */
void GpioControlInit(gpio_control_t *handle);

/**
 * @brief 配置 LED 闪烁周期、点亮占空比和重复次数。
 * @param handle GPIO 控制对象指针。
 * @param period 闪烁周期，单位为调度 tick。
 * @param light_on_percent 有效电平占空比，范围 0.0f 到 1.0f。
 * @param times 需要执行的闪烁周期次数。
 * @retval 无。
 */
void LedSetup(gpio_control_t *handle, uint32_t period, float light_on_percent, uint16_t times);

/**
 * @brief LED 周期任务入口。
 * @param 无。
 * @retval 无。
 */
void TaskLedProc(void);

extern gpio_control_t led1;

#ifdef __cplusplus
}
#endif

#endif /* INC_002_G_MYGPIO_H */

