/**
 * @file bsp_ringbuffer.c
 * @author Ws
 * @brief 环形缓冲区实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */


#include "bsp_ringbuffer.h"
#include <stdio.h>

/**
 * @brief 初始化环形缓冲区并清空存储区。
 * @param handle 环形缓冲区实例指针。
 * @retval 无。
 */

void RingbufferInit(ringbuffer_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    handle->r = 0U;
    handle->w = 0U;
    memset(handle->buffer, 0, sizeof(uint8_t) * RINGBUFFER_SIZE);
    handle->item_count = 0U;
}

/**
 * @brief 判断环形缓冲区是否已满。
 * @param handle 环形缓冲区实例指针。
 * @retval 1 表示环形缓冲区已满。
 * @retval 0 表示环形缓冲区仍有可用空间。
 */

uint8_t RingbufferIsFull(ringbuffer_t *handle)
{
    if (handle == NULL)
    {
        return 0U;
    }

    return (uint8_t)(handle->item_count == RINGBUFFER_SIZE);
}

/**
 * @brief 判断环形缓冲区是否为空。
 * @param handle 环形缓冲区实例指针。
 * @retval 1 表示环形缓冲区为空。
 * @retval 0 表示环形缓冲区中有数据。
 */

uint8_t RingbufferIsEmpty(ringbuffer_t *handle)
{
    if (handle == NULL)
    {
        return 1U;
    }

    return (uint8_t)(handle->item_count == 0U);
}

/**
 * @brief 向环形缓冲区写入数据。
 * @param handle 环形缓冲区实例指针。
 * @param data 待写入数据缓冲区指针。
 * @param num 需要写入的字节数。
 * @retval 0 写入成功。
 * @retval -1 写入前环形缓冲区已满或参数无效。
 */

int8_t RingbufferWrite(ringbuffer_t *handle, uint8_t *data, uint32_t num)
{
    uint32_t free_count;

    if ((handle == NULL) || (data == NULL))
    {
        return -1;
    }

    if (RingbufferIsFull(handle) != 0U)
    {
        return -1;
    }

    free_count = RINGBUFFER_SIZE - handle->item_count;
    if (num > free_count)
    {
        num = free_count;
    }

    while (num > 0U)
    {
        handle->buffer[handle->w] = *data;
        data++;
        handle->w = (handle->w + 1U) % RINGBUFFER_SIZE;
        handle->item_count++;
        num--;
    }

    return 0;
}

/**
 * @brief 从环形缓冲区读取数据。
 * @param handle 环形缓冲区实例指针。
 * @param data 读取数据的目标缓冲区指针。
 * @param num 需要读取的字节数。
 * @retval 0 读取成功。
 * @retval -1 读取前环形缓冲区为空或参数无效。
 */

int8_t RingbufferRead(ringbuffer_t *handle, uint8_t *data, uint32_t num)
{
    if ((handle == NULL) || (data == NULL))
    {
        return -1;
    }

    if (RingbufferIsEmpty(handle) != 0U)
    {
        return -1;
    }

    while (num > 0U)
    {
        *data = handle->buffer[handle->r];
        data++;
        handle->r = (handle->r + 1U) % RINGBUFFER_SIZE;
        handle->item_count--;
        num--;
    }

    return 0;
}

