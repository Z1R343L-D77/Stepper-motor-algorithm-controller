/**
 * @file drv_button.h
 * @author Ws
 * @brief 按键驱动对外接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef INC_002_G_MYBUTTON_H
#define INC_002_G_MYBUTTON_H

#include <stdint.h>
#include <string.h>

#define TICKS_INTERVAL    (5U)
#define DEBOUNCE_TICKS    (3U)
#define SHORT_TICKS       (300U / TICKS_INTERVAL)
#define LONG_TICKS        (1000U / TICKS_INTERVAL)

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*button_callback_t)(void *context);

typedef enum
{
    PRESS_DOWN = 0U,
    PRESS_UP,
    PRESS_REPEAT,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS_START,
    LONG_PRESS_HOLD,
    BUTTON_EVENT_COUNT,
    NONE_PRESS
} press_event_e;

typedef struct button
{
    uint16_t ticks;
    uint8_t repeat : 4;
    uint8_t event : 4;
    uint8_t state : 3;
    uint8_t debounce_cnt : 3;
    uint8_t active_level : 1;
    uint8_t button_level : 1;
    uint8_t button_id;
    uint8_t (*hal_button_level)(uint8_t button_id);
    button_callback_t cb[BUTTON_EVENT_COUNT];
    struct button *next;
} button_t;

/**
 * @brief 初始化按键句柄并绑定 GPIO 电平读取函数。
 * @param handle 按键控制句柄指针。
 * @param pin_level 根据按键 ID 读取物理 GPIO 电平的函数指针。
 * @param active_level 表示按键按下的有效 GPIO 电平。
 * @param button_id 传递给 pin_level 的逻辑按键 ID。
 * @retval 无。
 */
void ButtonInit(button_t *handle, uint8_t (*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id);

/**
 * @brief 为指定按键事件绑定回调函数。
 * @param handle 按键控制句柄指针。
 * @param event 需要绑定的按键事件类型。
 * @param cb 事件触发时调用的回调函数。
 * @retval 无。
 */
void ButtonAttach(button_t *handle, press_event_e event, button_callback_t cb);

/**
 * @brief 获取按键句柄中记录的最新事件。
 * @param handle 按键控制句柄指针。
 * @retval 当前按键事件。
 */
press_event_e GetButtonEvent(button_t *handle);

/**
 * @brief 将按键句柄加入活动轮询链表。
 * @param handle 需要启动的按键控制句柄指针。
 * @retval 0 表示添加成功。
 * @retval -1 表示句柄已存在于链表中。
 */
int ButtonStart(button_t *handle);

/**
 * @brief 从活动轮询链表中移除按键句柄。
 * @param handle 需要停止的按键控制句柄指针。
 * @retval 无。
 */
void ButtonStop(button_t *handle);

/**
 * @brief 轮询并处理已启动的按键链表。
 * @param 无。
 * @retval 无。
 */
void ButtonTicks(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_002_G_MYBUTTON_H */

