/**
 * @file bsp_key.c
 * @author Ws
 * @brief 板级按键支持实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "bsp_key.h"
#include "config.h"
#include <stdio.h>

typedef enum
{
    BUTTON_ID_1 = 0,
    BUTTON_ID_COUNT
} button_id_t;

button_t btn1;

/**
 * @brief 读取指定逻辑按键对应的物理 GPIO 电平。
 * @param button_id 逻辑按键 ID。
 * @retval GPIO_PIN_SET 选中的输入引脚为高电平。
 * @retval GPIO_PIN_RESET 选中的输入引脚为低电平或按键 ID 不支持。
 */

static uint8_t ReadButtonGpio(uint8_t button_id)
{
    switch (button_id)
    {
        case BUTTON_ID_1:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

        default:
            return 0U;
    }
}

/**
 * @brief 根据按键事件分发应用层动作。
 * @param context 按键驱动传入的上下文指针。
 * @retval 无。
 */

static void ButtonCallback(void *context)
{
    button_t *handle = (button_t *)context;

    if (handle == NULL)
    {
        return;
    }

    switch (handle->event)
    {
        case PRESS_DOWN:
            LedSetup(&led1, 100U, 0.5f, 1U);
            break;

        case PRESS_UP:
            break;

        case PRESS_REPEAT:
            break;

        case SINGLE_CLICK:
            break;

        case DOUBLE_CLICK:
            break;

        case LONG_PRESS_START:
            break;

        case LONG_PRESS_HOLD:
            break;

        default:
            break;
    }
}

/**
 * @brief 初始化板级按键句柄并注册事件回调。
 * @param 无。
 * @retval 无。
 */

void ButtonInitAll(void)
{
    ButtonInit(&btn1, ReadButtonGpio, KEY_LEVEL_HIGH, BUTTON_ID_1);

    ButtonAttach(&btn1, PRESS_DOWN, ButtonCallback);
    ButtonAttach(&btn1, SINGLE_CLICK, ButtonCallback);
    ButtonAttach(&btn1, DOUBLE_CLICK, ButtonCallback);
    ButtonAttach(&btn1, LONG_PRESS_START, ButtonCallback);
    ButtonAttach(&btn1, LONG_PRESS_HOLD, ButtonCallback);

    ButtonStart(&btn1);
}

