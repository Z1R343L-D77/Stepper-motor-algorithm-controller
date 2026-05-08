/**
 * @file drv_gpio.h
 * @author 19816
 * @brief GPIO驱动（LED控制）头文件
 * @version 1.2
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef DRV_GPIO_H
#define DRV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* 标准头文件 */
#include <stdint.h>

/* 工程头文件 */
#include "at32f403a_407_gpio.h"

/* 宏定义 */
#define GPIO_TYPEDEF            gpio_type
#define GPIO_ON(_LIGHT)         do { gpio_bits_write((_LIGHT)->port, (_LIGHT)->pin, TRUE); } while (0)
#define GPIO_OFF(_LIGHT)        do { gpio_bits_write((_LIGHT)->port, (_LIGHT)->pin, FALSE); } while (0)

/* 类型定义 */
typedef enum
{
    GPIO_LOW = 0U,
    GPIO_HIGH = 1U
} gpio_level_e;

typedef struct
{
    GPIO_TYPEDEF *port;
    uint32_t pin;
    uint32_t period;
    uint16_t total_times;
    uint16_t cycle_cnt;
    uint16_t completed_times;
    uint8_t reset_flag;
    uint8_t finish_flag;
    float duty_cycle;
    gpio_level_e active_level;
} gpio_control_t;

/* 函数声明 */
void GPIO_Init(void);
void GPIO_SetupLed1(uint32_t period, float duty_cycle, uint16_t total_times);
void GPIO_TaskProc(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_GPIO_H */

