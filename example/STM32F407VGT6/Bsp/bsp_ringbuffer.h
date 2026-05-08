/**
 * @file bsp_ringbuffer.h
 * @author Ws
 * @brief 环形缓冲区接口定义。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "config.h"

#define RINGBUFFER_SIZE  (128U)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t w;
    uint32_t r;
    uint8_t buffer[RINGBUFFER_SIZE];
    uint32_t item_count;
} ringbuffer_t;

/**
 * @brief 初始化环形缓冲区对象。
 * @param handle 环形缓冲区对象指针。
 * @retval 无。
 */
void RingbufferInit(ringbuffer_t *handle);

/**
 * @brief 判断环形缓冲区是否已满。
 * @param handle 环形缓冲区对象指针。
 * @retval 1 表示缓冲区已满。
 * @retval 0 表示缓冲区未满。
 */
uint8_t RingbufferIsFull(ringbuffer_t *handle);

/**
 * @brief 判断环形缓冲区是否为空。
 * @param handle 环形缓冲区对象指针。
 * @retval 1 表示缓冲区为空。
 * @retval 0 表示缓冲区非空。
 */
uint8_t RingbufferIsEmpty(ringbuffer_t *handle);

/**
 * @brief 向环形缓冲区写入指定长度的数据。
 * @param handle 环形缓冲区对象指针。
 * @param data 待写入数据缓冲区指针。
 * @param num 待写入字节数。
 * @retval 0 写入成功。
 * @retval -1 缓冲区空间不足或参数无效。
 */
int8_t RingbufferWrite(ringbuffer_t *handle, uint8_t *data, uint32_t num);

/**
 * @brief 从环形缓冲区读取指定长度的数据。
 * @param handle 环形缓冲区对象指针。
 * @param data 读取数据输出缓冲区指针。
 * @param num 待读取字节数。
 * @retval 0 读取成功。
 * @retval -1 缓冲区数据不足或参数无效。
 */
int8_t RingbufferRead(ringbuffer_t *handle, uint8_t *data, uint32_t num);

extern ringbuffer_t usart_rb;

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H */

