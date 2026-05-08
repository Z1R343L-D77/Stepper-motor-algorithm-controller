/**
 * @file bsp_uart.c
 * @author Ws
 * @brief UART 板级处理实现。
 * @version 1.0.0
 * @date 2026-05-06
 *
 * @copyright Copyright (c) 2026
 */

#include "bsp_uart.h"
#include "usart.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define UART_BUFFER_SIZE      (64U)
#define UART_CMD_BUFFER_SIZE  (96U)

ringbuffer_t usart_rb;
static uint8_t usart_read_buffer[UART_BUFFER_SIZE];

static char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
static uint16_t uart_cmd_len;

/**
 * @brief 将字符串解析为浮点数。
 * @param text 输入字符串。
 * @param value 解析后的输出值。
 * @retval 1 表示解析成功。
 * @retval 0 表示解析失败。
 */
static uint8_t UartParseFloat(const char *text, float *value)
{
    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    if (*text == '\0')
    {
        return 0U;
    }

    if (((*text < '0') || (*text > '9')) &&
        (*text != '-') &&
        (*text != '+') &&
        (*text != '.'))
    {
        return 0U;
    }

    *value = (float)atof(text);
    return 1U;
}

/**
 * @brief 打印当前步进电机状态。
 * @param 无。
 * @retval 无。
 */
static void UartPrintStatus(void)
{
    int32_t position_um = (int32_t)(StepperAppPositionMm() * 1000.0f);

    printf("OK STATUS BUSY=%u POS_UM=%ld REM=%lu\r\n",
           StepperAppIsBusy(),
           (long)position_um,
           (unsigned long)StepperAppRemainingPulses());
}

/**
 * @brief 处理一条完整 UART 命令。
 * @param line 待解析命令字符串。
 * @retval 无。
 */
static void UartHandleCommand(char *line)
{
    char *cmd;
    char *arg1;
    char *arg2;
    float value1;
    float value2;
    uint8_t ok;

    if (line == NULL)
    {
        return;
    }

    cmd = strtok(line, ", ");
    if (cmd == NULL)
    {
        return;
    }

    if (strcmp(cmd, "STATUS") == 0)
    {
        UartPrintStatus();
        return;
    }

    if (strcmp(cmd, "STOP") == 0)
    {
        ok = StepperAppStop();
        printf("%s STOP\r\n", (ok != 0U) ? "OK" : "ERR");
        return;
    }

    if (strcmp(cmd, "ZERO") == 0)
    {
        if (StepperAppIsBusy() != 0U)
        {
            printf("ERR ZERO BUSY\r\n");
            return;
        }

        StepperAppZeroPosition();
        printf("OK ZERO\r\n");
        return;
    }

    if (strcmp(cmd, "MOVE") == 0)
    {
        arg1 = strtok(NULL, ", ");
        arg2 = strtok(NULL, ", ");

        if ((UartParseFloat(arg1, &value1) == 0U) ||
            (UartParseFloat(arg2, &value2) == 0U))
        {
            printf("ERR MOVE ARG\r\n");
            return;
        }

        ok = StepperAppMoveMm(value1, value2);
        printf("%s MOVE\r\n", (ok != 0U) ? "OK" : "ERR");
        return;
    }

    if (strcmp(cmd, "CV") == 0)
    {
        arg1 = strtok(NULL, ", ");

        if (UartParseFloat(arg1, &value1) == 0U)
        {
            printf("ERR CV ARG\r\n");
            return;
        }

        ok = StepperAppSetSpeedMmS(value1);
        printf("%s CV\r\n", (ok != 0U) ? "OK" : "ERR");
        return;
    }

    printf("ERR UNKNOWN\r\n");
}

/**
 * @brief 按字节拼接 UART 命令并触发行解析。
 * @param byte 当前输入字节。
 * @retval 无。
 */
static void UartProcessByte(uint8_t byte)
{
    if ((byte == '\r') || (byte == '\n'))
    {
        if (uart_cmd_len != 0U)
        {
            uart_cmd_buffer[uart_cmd_len] = '\0';
            UartHandleCommand(uart_cmd_buffer);
            uart_cmd_len = 0U;
        }
        return;
    }

    if (uart_cmd_len < (UART_CMD_BUFFER_SIZE - 1U))
    {
        uart_cmd_buffer[uart_cmd_len] = (char)byte;
        uart_cmd_len++;
    }
    else
    {
        uart_cmd_len = 0U;
        printf("ERR CMD LEN\r\n");
    }
}

/**
 * @brief STM32 HAL UART 空闲接收 DMA 事件回调函数。
 * @param handle STM32 HAL 传入的 UART 句柄指针。
 * @param size uart_rx_dma_buffer 中本次接收到的字节数。
 * @retval 无。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    (void)handle;

    if (RingbufferIsFull(&usart_rb) == 0U)
    {
        RingbufferWrite(&usart_rb, uart_rx_dma_buffer, size);
    }

    memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer, sizeof(uart_rx_dma_buffer));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief 处理 UART 接收环形缓冲区中的数据。
 * @param 无。
 * @retval 无。
 */
void UartProc(void)
{
    uint32_t read_len;
    uint32_t i;

    if (RingbufferIsEmpty(&usart_rb) != 0U)
    {
        return;
    }

    read_len = usart_rb.item_count;
    if (read_len > UART_BUFFER_SIZE)
    {
        read_len = UART_BUFFER_SIZE;
    }

    RingbufferRead(&usart_rb, usart_read_buffer, read_len);

    for (i = 0U; i < read_len; i++)
    {
        UartProcessByte(usart_read_buffer[i]);
    }

    memset(usart_read_buffer, 0, sizeof(uint8_t) * UART_BUFFER_SIZE);
}

