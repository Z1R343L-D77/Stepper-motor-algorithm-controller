/**
 * @file bsp_key.h
 * @author Ws
 * @brief 板级按键接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_002_G_BSP_KEY_H
#define INC_002_G_BSP_KEY_H

#include "drv_button.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
    KEY_LEVEL_LOW = 0,
    KEY_LEVEL_HIGH
} key_level_t;

/**
 * @brief 初始化板级按键句柄并注册事件回调。
 * @param 无。
 * @retval 无。
 */
void ButtonInitAll(void);

extern button_t btn1;

#ifdef __cplusplus
}
#endif

#endif /* INC_002_G_BSP_KEY_H */

