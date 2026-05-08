/**
 * @file drv_goseiko.h
 * @author 19816
 * @brief 电感式金属传感器驱动头文件
 * @version 1.2
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef DRV_GOSEIKO_H
#define DRV_GOSEIKO_H

#ifdef __cplusplus
extern "C" {
#endif

/* 标准头文件 */
#include <stdint.h>

/* 工程头文件 */
#include "at32f403a_407_gpio.h"

/* 宏定义 */
#define GPIO_TYPEDEF            gpio_type
#define GOSEIKO_READ(_S)        gpio_input_data_bit_read((_S)->port, (_S)->pin)
#define DEBOUNCE_THRESHOLD      3U

/* 类型定义 */
typedef enum
{
    GOSEIKO_NO_METAL = 0U,
    GOSEIKO_DETECTED = 1U
} goseiko_state_e;

typedef enum
{
    GOSEIKO_X1 = 0U,
    GOSEIKO_X2 = 1U,
    GOSEIKO_X3 = 2U,
    GOSEIKO_MAX = 3U
} goseiko_id_e;

typedef struct
{
    GPIO_TYPEDEF *port;
    uint32_t pin;
    goseiko_state_e current_state;
    goseiko_state_e last_raw_state;
    uint8_t debounce_cnt;
    uint8_t debounce_threshold;
} goseiko_control_t;

/* 函数声明 */
void GOSEIKO_Init(void);
goseiko_state_e GOSEIKO_ReadRaw(goseiko_id_e goseiko_id);
uint8_t GOSEIKO_GetState(goseiko_id_e goseiko_id);
void GOSEIKO_TaskScanProc(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_GOSEIKO_H */

