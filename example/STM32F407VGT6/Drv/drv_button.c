/**
 * @file drv_button.c
 * @author Ws
 * @brief 按键驱动状态机实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "drv_button.h"
#include <stdio.h>

#define EVENT_CB(ev) \
    do \
    { \
        if ((handle->cb[(ev)]) != NULL) \
        { \
            handle->cb[(ev)]((void *)handle); \
        } \
    } while (0)
	
#define PRESS_REPEAT_MAX_NUM  (15U)

static button_t *head_handle = NULL;

static void ButtonHandler(button_t *handle);

/**
 * @brief 初始化按键句柄并绑定 GPIO 电平读取函数。
 * @param handle 按键控制句柄指针。
 * @param pin_level 根据按键 ID 读取物理 GPIO 电平的函数指针。
 * @param active_level 表示按键按下的有效 GPIO 电平。
 * @param button_id 传递给 pin_level 的逻辑按键 ID。
 * @retval 无。
 */

void ButtonInit(button_t *handle, uint8_t (*pin_level)(uint8_t), uint8_t active_level, uint8_t button_id)
{
    if ((handle == NULL) || (pin_level == NULL))
    {
        return;
    }

    memset(handle, 0, sizeof(button_t));
    handle->event = (uint8_t)NONE_PRESS;
    handle->hal_button_level = pin_level;
    handle->button_level = (uint8_t)(!active_level);
    handle->active_level = active_level;
    handle->button_id = button_id;
}

/**
 * @brief 为指定按键事件绑定回调函数。
 * @param handle 按键控制句柄指针。
 * @param event 需要绑定的按键事件类型。
 * @param cb 事件触发时调用的回调函数。
 * @retval 无。
 */

void ButtonAttach(button_t *handle, press_event_e event, button_callback_t cb)
{
    if (handle == NULL)
    {
        return;
    }

    handle->cb[event] = cb;
}

/**
 * @brief 获取按键句柄中记录的最新事件。
 * @param handle 按键控制句柄指针。
 * @retval 当前按键事件。
 */

press_event_e GetButtonEvent(button_t *handle)
{
    if (handle == NULL)
    {
        return NONE_PRESS;
    }

    return (press_event_e)(handle->event);
}

/**
 * @brief 执行单个按键的消抖和状态机处理。
 * @param handle 按键控制句柄指针。
 * @retval 无。
 */

static void ButtonHandler(button_t *handle)
{
    uint8_t read_gpio_level;

    if (handle == NULL)
    {
        return;
    }

    read_gpio_level = handle->hal_button_level(handle->button_id);

    if ((handle->state) > 0U)
    {
        handle->ticks++;
    }

    if (read_gpio_level != handle->button_level)
    {
        handle->debounce_cnt++;
        if (handle->debounce_cnt >= DEBOUNCE_TICKS)
        {
            handle->button_level = read_gpio_level;
            handle->debounce_cnt = 0U;
        }
    }
    else
    {
        handle->debounce_cnt = 0U;
    }

    switch (handle->state)
    {
        case 0U:
            if (handle->button_level == handle->active_level)
            {
                handle->event = (uint8_t)PRESS_DOWN;
                EVENT_CB(PRESS_DOWN);
                handle->ticks = 0U;
                handle->repeat = 1U;
                handle->state = 1U;
            }
            else
            {
                handle->event = (uint8_t)NONE_PRESS;
            }
            break;

        case 1U:
            if (handle->button_level != handle->active_level)
            {
                handle->event = (uint8_t)PRESS_UP;
                EVENT_CB(PRESS_UP);
                handle->ticks = 0U;
                handle->state = 2U;
            }
            else if (handle->ticks > LONG_TICKS)
            {
                handle->event = (uint8_t)LONG_PRESS_START;
                EVENT_CB(LONG_PRESS_START);
                handle->state = 5U;
            }
            else
            {
                /* 状态保持，不需要额外处理。 */

            }
            break;

        case 2U:
            if (handle->button_level == handle->active_level)
            {
                handle->event = (uint8_t)PRESS_DOWN;
                EVENT_CB(PRESS_DOWN);
                if (handle->repeat != PRESS_REPEAT_MAX_NUM)
                {
                    handle->repeat++;
                }
                EVENT_CB(PRESS_REPEAT);
                handle->ticks = 0U;
                handle->state = 3U;
            }
            else if (handle->ticks > SHORT_TICKS)
            {
                if (handle->repeat == 1U)
                {
                    handle->event = (uint8_t)SINGLE_CLICK;
                    EVENT_CB(SINGLE_CLICK);
                }
                else if (handle->repeat == 2U)
                {
                    handle->event = (uint8_t)DOUBLE_CLICK;
                    EVENT_CB(DOUBLE_CLICK);
                }
                else
                {
                    /* 大于双击次数时不触发额外短按事件。 */

                }
                handle->state = 0U;
            }
            else
            {
                /* 等待超时或下一次按下。 */

            }
            break;

        case 3U:
            if (handle->button_level != handle->active_level)
            {
                handle->event = (uint8_t)PRESS_UP;
                EVENT_CB(PRESS_UP);
                if (handle->ticks < SHORT_TICKS)
                {
                    handle->ticks = 0U;
                    handle->state = 2U;
                }
                else
                {
                    handle->state = 0U;
                }
            }
            else if (handle->ticks > SHORT_TICKS)
            {
                handle->state = 1U;
            }
            else
            {
                /* 继续等待释放或长按判定。 */

            }
            break;

        case 5U:
            if (handle->button_level == handle->active_level)
            {
                handle->event = (uint8_t)LONG_PRESS_HOLD;
                EVENT_CB(LONG_PRESS_HOLD);
            }
            else
            {
                handle->event = (uint8_t)PRESS_UP;
                EVENT_CB(PRESS_UP);
                handle->state = 0U;
            }
            break;

        default:
            handle->state = 0U;
            break;
    }
}

/**
 * @brief 将按键句柄加入活动轮询链表。
 * @param handle 需要启动的按键控制句柄指针。
 * @retval 0 表示句柄添加成功。
 * @retval -1 表示句柄已存在于轮询链表中。
 */

int ButtonStart(button_t *handle)
{
    button_t *target;

    if (handle == NULL)
    {
        return -1;
    }

    target = head_handle;
    while (target != NULL)
    {
        if (target == handle)
        {
            return -1;
        }
        target = target->next;
    }

    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

/**
 * @brief 从活动轮询链表中移除按键句柄。
 * @param handle 需要停止的按键控制句柄指针。
 * @retval 无。
 */

void ButtonStop(button_t *handle)
{
    button_t **curr;

    if (handle == NULL)
    {
        return;
    }

    for (curr = &head_handle; (*curr) != NULL; )
    {
        button_t *entry = *curr;

        if (entry == handle)
        {
            *curr = entry->next;
            return;
        }
        else
        {
            curr = &entry->next;
        }
    }
}

/**
 * @brief 轮询并处理已启动的按键链表。
 * @param 无。
 * @retval 无。
 */

void ButtonTicks(void)
{
    button_t *target;

    for (target = head_handle; target != NULL; target = target->next)
    {
        ButtonHandler(target);
    }
}

