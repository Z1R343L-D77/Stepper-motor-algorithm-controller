/**
 * @file bsp_led.c
 * @author 19816
 * @brief LED 板级支持包源文件
 * @version 1.1
 * @date 2026-05-07
 *
 * @copyright Copyright (c) 2026
 */

#include "drv_gpio.h"
#include "bsp_led.h"
#include "at32f403a_407_gpio.h"

/**
 * @brief 熄灭 LED
 * @param 无
 * @retval 无
 */
static void BSP_LED_Off(void)
{
    /* 直接输出高电平，使低电平点亮型 LED 熄灭 */
    gpio_bits_set(GPIOB, GPIO_PINS_3);
}

/**
 * @brief 初始化 LED（PB3）
 * @param 无
 * @retval 无
 */
void BSP_LED_Init(void)
{
    gpio_init_type gpio_init_struct;

    /* 使能 GPIOB 与 IOMUX 时钟，准备引脚复用配置 */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    /* 禁用 JTAG，释放 PB3（JTDO）用于 LED 输出 */
    gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);

    /* 将 PB3 配置为推挽输出，驱动 LED 引脚 */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio_init_struct);

    /* 设置初始输出状态为熄灭，避免上电闪烁 */
    BSP_LED_Off();

    /* 初始化 GPIO 驱动层状态机参数 */
    GPIO_Init();
}

