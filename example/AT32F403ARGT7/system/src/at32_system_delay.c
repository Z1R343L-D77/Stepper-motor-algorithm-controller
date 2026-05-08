#include "at32f403a_407.h"

static void delay_cycles(volatile uint32_t cycles)
{
    while(cycles--)
    {
        __asm("nop");
    }
}

void delay_us(uint32_t us)
{
    while(us--)
    {
        delay_cycles(240);
    }
}

void delay_ms(uint32_t ms)
{
    while(ms--)
    {
        delay_us(1000);
    }
}
