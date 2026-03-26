/**
 * @file drv_step.c
 * @brief 步进电机脉冲生成驱动程序
 * @details 实现步进电机的梯形/S 曲线速度控制，支持：
 *          1. Tacc 自动规划模式
 *          2. 固定脉冲分段模式（加速/匀速/减速）
 */

#include "drv_step.h"
#include "stdio.h"
#include "math.h"

#ifndef M_PI
#define M_PI (3.1415926535f)
#endif

/**
 * @brief 将频率转换为定时器 ARR/PSC 对应值
 * @param hstep 步进电机控制句柄指针
 * @param freq  目标频率
 * @return 定时器比较值
 */
static uint16_t Step_FreqToPsc(stepTypedef *hstep, float freq)
{
    if (freq < hstep->Fmin)
        freq = hstep->Fmin;

    if (freq > hstep->Fmax)
        freq = hstep->Fmax;

    return (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / freq - 1);
}

/**
 * @brief 初始化步进电机控制结构体
 * @param hstep 步进电机控制句柄指针
 * @param tmr 定时器指针
 * @param channel 定时器通道选择
 * @param GPIOx GPIO 端口指针
 * @param GPIO_Pin GPIO 引脚编号
 * @param Fmin 最小频率 (Hz)
 * @param Fmax 最大频率 (Hz)
 * @param Tacc 加速时间 (ms)
 */
void Step_Init(stepTypedef* hstep,
               tmr_type* tmr,
               tmr_channel_select_type channel,
               gpio_type* GPIOx,
               uint16_t GPIO_Pin,
               float Fmin,
               float Fmax,
               float Tacc)
{
    hstep->tmr = tmr;
    hstep->channel = channel;
    hstep->gpioPort = GPIOx;
    hstep->gpioPin = GPIO_Pin;

    /* 设置 PWM 占空比为 50% */
    {
        uint32_t arr = tmr->pr;
        tmr_channel_value_set(tmr, channel, arr / 2);
    }

    hstep->Fmin = Fmin;
    hstep->Fmax = Fmax;
    hstep->Tacc = Tacc;

    hstep->t = 0.0f;
    hstep->Fcur = Fmin;

    hstep->state = Stop;
    hstep->lock = UNLOCK;

    hstep->stepToGo = 0;
    hstep->buffToUse = 0;
    hstep->accStep = 0;
    hstep->pos = 0;

    /* 固定分段模式参数初始化 */
    hstep->totalPulse = 0;
    hstep->accPulse = 0;
    hstep->constPulse = 0;
    hstep->decPulse = 0;

    hstep->accCount = 0;
    hstep->constCount = 0;
    hstep->decCount = 0;

    hstep->useDec = 1;
    hstep->buffIndex = 0;
    hstep->buffRdy = 0;
    hstep->dir = 1;
    hstep->motionMode = 0;
    hstep->flag = 0;
    hstep->fixedMode = 0;

#if AutoInitBuffer
    for (int i = 0; i < BufferSize; i++)
    {
        hstep->buff0[i] = 0;
        hstep->buff1[i] = 0;
    }
#endif
}

/**
 * @brief 填充加速阶段脉冲缓冲区（ Tacc 模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillAccelerate(stepTypedef* hstep)
{
    if (hstep->state != Acclerate)
        return 0;

    int buffToUse = (hstep->stepToGo > BufferSize) ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    int i = 0;

    for (; hstep->t <= hstep->Tacc; i++)
    {
        if (i >= buffToUse)
            break;

        /* 用上一拍频率推时间 */
        hstep->t += 1000.0f / hstep->Fcur;

        switch (AcclerateCurve)
        {
        case Curve_Trapezoidal:
            hstep->Fcur = (hstep->Fmax - hstep->Fmin) / hstep->Tacc * hstep->t + hstep->Fmin;
            break;

        case Curve_S:
            hstep->Fcur = 0.5f * hstep->Fmax * cosf(M_PI - M_PI * hstep->t / hstep->Tacc)
                        + hstep->Fmin + hstep->Fmax * 0.5f;
            break;

        default:
            break;
        }

        if (hstep->Fcur > hstep->Fmax)
            hstep->Fcur = hstep->Fmax;

        buffPtr[i] = Step_FreqToPsc(hstep, hstep->Fcur);
    }

    hstep->buffIndex = !hstep->buffIndex;

    if (i >= buffToUse)
    {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    /* 加速结束，补齐为匀速 */
    hstep->state = Constant;
    hstep->Fcur = hstep->Fmax;

    {
        uint16_t PSC = Step_FreqToPsc(hstep, hstep->Fcur);
        for (; i < buffToUse; i++)
        {
            buffPtr[i] = PSC;
        }
    }

    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 填充匀速阶段脉冲缓冲区（ Tacc 模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillConstant(stepTypedef* hstep)
{
    if (hstep->state != Constant)
        return 0;

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    uint16_t PSC = Step_FreqToPsc(hstep, hstep->Fcur);
    int buffToUse = (hstep->stepToGo > BufferSize) ? BufferSize : hstep->stepToGo;

    for (int i = 0; i < buffToUse; i++)
    {
        buffPtr[i] = PSC;
    }

    hstep->buffIndex = !hstep->buffIndex;
    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 填充减速阶段脉冲缓冲区（ Tacc 模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillDecelerate(stepTypedef* hstep)
{
    if (hstep->state != Decelerate)
        return 0;

    int buffToUse = (hstep->stepToGo > BufferSize) ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    int i = 0;

    for (; hstep->Fcur > hstep->Fmin; i++)
    {
        if (i >= buffToUse)
            break;

        hstep->t -= 1000.0f / hstep->Fcur;
        hstep->Fcur = (hstep->Fmax - hstep->Fmin) / hstep->Tacc * hstep->t + hstep->Fmin;

        if (hstep->Fcur < hstep->Fmin)
            hstep->Fcur = hstep->Fmin;

        buffPtr[i] = Step_FreqToPsc(hstep, hstep->Fcur);
    }

    hstep->buffIndex = !hstep->buffIndex;

    if (i >= buffToUse)
    {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    /* 减速末尾补最小频率 */
    hstep->Fcur = hstep->Fmin;
    {
        uint16_t PSC = Step_FreqToPsc(hstep, hstep->Fcur);
        for (; i < buffToUse; i++)
        {
            buffPtr[i] = PSC;
        }
    }

    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 填充加速阶段脉冲缓冲区（固定分段模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillAccelerateFixed(stepTypedef* hstep)
{
    if (hstep->state != Acclerate)
        return 0;

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    uint32_t remainAcc = hstep->accPulse - hstep->accCount;
    uint32_t buffToUse = (remainAcc > BufferSize) ? BufferSize : remainAcc;

    if (buffToUse > hstep->stepToGo)
        buffToUse = hstep->stepToGo;

    for (uint32_t i = 0; i < buffToUse; i++)
    {
        float ratio;

        if (hstep->accPulse <= 1)
        {
            ratio = 1.0f;
        }
        else
        {
            ratio = (float)(hstep->accCount + i) / (float)(hstep->accPulse - 1);
        }

        switch (AcclerateCurve)
        {
        case Curve_Trapezoidal:
            hstep->Fcur = hstep->Fmin + (hstep->Fmax - hstep->Fmin) * ratio;
            break;

        case Curve_S:
            hstep->Fcur = hstep->Fmin
                        + (hstep->Fmax - hstep->Fmin)
                        * (0.5f - 0.5f * cosf(M_PI * ratio));
            break;

        default:
            hstep->Fcur = hstep->Fmin + (hstep->Fmax - hstep->Fmin) * ratio;
            break;
        }

        buffPtr[i] = Step_FreqToPsc(hstep, hstep->Fcur);
    }

    hstep->accCount += buffToUse;
    hstep->buffIndex = !hstep->buffIndex;
    hstep->buffRdy = 1;

    if (hstep->accCount >= hstep->accPulse)
    {
        hstep->state = Constant;
        hstep->Fcur = hstep->Fmax;
    }

    return buffToUse;
}

/**
 * @brief 填充匀速阶段脉冲缓冲区（固定分段模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillConstantFixed(stepTypedef* hstep)
{
    if (hstep->state != Constant)
        return 0;

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    uint32_t remainConst = hstep->constPulse - hstep->constCount;
    uint32_t buffToUse = (remainConst > BufferSize) ? BufferSize : remainConst;

    if (buffToUse > hstep->stepToGo)
        buffToUse = hstep->stepToGo;

    hstep->Fcur = hstep->Fmax;
    {
        uint16_t PSC = Step_FreqToPsc(hstep, hstep->Fcur);
        for (uint32_t i = 0; i < buffToUse; i++)
        {
            buffPtr[i] = PSC;
        }
    }

    hstep->constCount += buffToUse;
    hstep->buffIndex = !hstep->buffIndex;
    hstep->buffRdy = 1;

    if (hstep->constCount >= hstep->constPulse)
    {
        hstep->state = Decelerate;
    }

    return buffToUse;
}

/**
 * @brief 填充减速阶段脉冲缓冲区（固定分段模式）
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillDecelerateFixed(stepTypedef* hstep)
{
    if (hstep->state != Decelerate)
        return 0;

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    uint32_t remainDec = hstep->decPulse - hstep->decCount;
    uint32_t buffToUse = (remainDec > BufferSize) ? BufferSize : remainDec;

    if (buffToUse > hstep->stepToGo)
        buffToUse = hstep->stepToGo;

    for (uint32_t i = 0; i < buffToUse; i++)
    {
        float ratio;

        if (hstep->decPulse <= 1)
        {
            ratio = 1.0f;
        }
        else
        {
            ratio = (float)(hstep->decCount + i) / (float)(hstep->decPulse - 1);
        }

        switch (AcclerateCurve)
        {
        case Curve_Trapezoidal:
            hstep->Fcur = hstep->Fmax - (hstep->Fmax - hstep->Fmin) * ratio;
            break;

        case Curve_S:
            hstep->Fcur = hstep->Fmax
                        - (hstep->Fmax - hstep->Fmin)
                        * (0.5f - 0.5f * cosf(M_PI * ratio));
            break;

        default:
            hstep->Fcur = hstep->Fmax - (hstep->Fmax - hstep->Fmin) * ratio;
            break;
        }

        if (hstep->Fcur < hstep->Fmin)
            hstep->Fcur = hstep->Fmin;

        buffPtr[i] = Step_FreqToPsc(hstep, hstep->Fcur);
    }

    hstep->decCount += buffToUse;
    hstep->buffIndex = !hstep->buffIndex;
    hstep->buffRdy = 1;

    if (hstep->decCount >= hstep->decPulse)
    {
        hstep->state = Stop;
        hstep->Fcur = hstep->Fmin;
    }

    return buffToUse;
}

/**
 * @brief 缓冲区使用完成回调
 * @param hstep 步进电机控制句柄指针
 * @details 更新剩余步数，判断是否进入减速阶段或停止
 */
void Step_BufferUsed(stepTypedef* hstep)
{
    hstep->buffRdy = 0;

    if (hstep->stepToGo >= hstep->buffToUse)
        hstep->stepToGo -= hstep->buffToUse;
    else
        hstep->stepToGo = 0;

    if (hstep->dir)
        hstep->pos += hstep->buffToUse;
    else
        hstep->pos -= hstep->buffToUse;

    if (hstep->stepToGo == 0)
    {
        hstep->state = Stop;
        Step_Unlock(hstep);
        return;
    }

    /*  Tacc 模式下仍按原逻辑切换减速 */
    if (hstep->fixedMode == 0)
    {
        if (hstep->stepToGo <= hstep->accStep)
        {
            if (hstep->useDec)
                hstep->state = Decelerate;
            else
                hstep->state = Constant;
        }
    }
}

/**
 * @brief 检查缓冲区是否就绪
 * @param hstep 步进电机控制句柄指针
 * @return 1-就绪，0-未就绪
 */
int Step_IsBuffRdy(stepTypedef* hstep)
{
    return hstep->buffRdy;
}

/**
 * @brief 获取当前缓冲区指针
 * @param hstep 步进电机控制句柄指针
 * @return 当前缓冲区地址
 */
uint16_t* Step_GetCurBuffer(stepTypedef* hstep)
{
    return hstep->buffIndex ? hstep->buff0 : hstep->buff1;
}

/**
 * @brief 获取缓冲区使用长度
 * @param hstep 步进电机控制句柄指针
 * @return 已使用的缓冲区长度
 */
uint32_t Step_BuffUsedLength(stepTypedef* hstep)
{
    return hstep->buffToUse;
}

/**
 * @brief 锁定步进电机控制
 * @param hstep 步进电机控制句柄指针
 * @return 0-成功，-1-已锁定
 */
int Step_Lock(stepTypedef* hstep)
{
    if (hstep->lock == UNLOCK)
    {
        hstep->lock = LOCK;
    }
    else
    {
        return -1;
    }

    return 0;
}

/**
 * @brief 解锁步进电机控制
 * @param hstep 步进电机控制句柄指针
 * @return 0-成功，-1-已解锁
 */
int Step_Unlock(stepTypedef* hstep)
{
    if (hstep->lock == LOCK)
    {
        hstep->lock = UNLOCK;
    }
    else
    {
        return -1;
    }

    return 0;
}

/**
 * @brief 中止步进电机运行
 * @param hstep 步进电机控制句柄指针
 */
void Step_Abort(stepTypedef* hstep)
{
    tmr_counter_enable(hstep->tmr, FALSE);
    hstep->state = Stop;
    hstep->buffRdy = 0;
    hstep->stepToGo = 0;
    Step_Unlock(hstep);
}

/**
 * @brief 预填充缓冲区，启动步进电机（原有接口）
 * @param hstep 步进电机控制句柄指针
 * @param stepToGo 目标步数
 * @param dir 方向 (1-正，0-反)
 * @param useDec 是否使用减速 (1-是，0-否)
 * @return 填充的脉冲数量
 */
int Step_Prefill(stepTypedef* hstep,
                 int stepToGo,
                 uint8_t dir,
                 uint8_t useDec)
{
    if (stepToGo <= 0)
        return -1;

    if (Step_Lock(hstep) != 0)
        return -1;

    hstep->fixedMode = 0;
    hstep->stepToGo = (uint32_t)stepToGo;
    hstep->useDec = useDec;
    hstep->dir = dir;
    hstep->buffToUse = 0;
    hstep->t = 0.0f;
    hstep->Fcur = hstep->Fmin;

    /* Tacc 模式下，根据加速曲线计算加速步数 */ 
    switch (AcclerateCurve)
    {
    case Curve_Trapezoidal:
        hstep->accStep = (uint32_t)((hstep->Fmin + hstep->Fmax) * hstep->Tacc / 2000.0f);
        break;

    case Curve_S:
        hstep->accStep = (uint32_t)(0.5f * hstep->Fmax * hstep->Tacc * 0.001f
                         + hstep->Fmin * hstep->Tacc * 0.001f);
        break;

    default:
        hstep->accStep = 0;
        break;
    }

    if (dir)
        gpio_bits_set(hstep->gpioPort, hstep->gpioPin);
    else
        gpio_bits_reset(hstep->gpioPort, hstep->gpioPin);

    if (hstep->Tacc == 0)
    {
        hstep->Fcur = hstep->Fmax;
        hstep->state = Constant;
        hstep->buffToUse = Step_FillConstant(hstep);
    }
    else if (hstep->accStep > (uint32_t)stepToGo)
    {
        /* 步数太短时直接匀速 */
        hstep->Fcur = hstep->Fmin;
        hstep->state = Constant;
        hstep->buffToUse = Step_FillConstant(hstep);
    }
    else
    {
        hstep->state = Acclerate;
        hstep->buffToUse = Step_FillAccelerate(hstep);
    }

    return hstep->buffToUse;
}

/**
 * @brief 预填充缓冲区，启动步进电机（固定脉冲分段模式）
 * @param hstep 步进电机控制句柄指针
 * @param totalPulse 总脉冲数
 * @param accPulse 加速段脉冲数
 * @param constPulse 匀速段脉冲数
 * @param decPulse 减速段脉冲数
 * @param dir 方向 (1-正，0-反)
 * @return 填充的脉冲数量，-1-参数错误
 */
int Step_PrefillFixed(stepTypedef *hstep,
                      uint32_t totalPulse,
                      uint32_t accPulse,
                      uint32_t constPulse,
                      uint32_t decPulse,
                      uint8_t dir)
{
    if (totalPulse == 0)
        return -1;

    if ((accPulse + constPulse + decPulse) != totalPulse)
        return -1;

    if (Step_Lock(hstep) != 0)
        return -1;

    hstep->fixedMode = 1;

    hstep->totalPulse = totalPulse;
    hstep->stepToGo   = totalPulse;

    hstep->accPulse   = accPulse;
    hstep->constPulse = constPulse;
    hstep->decPulse   = decPulse;

    hstep->accCount   = 0;
    hstep->constCount = 0;
    hstep->decCount   = 0;

    hstep->dir = dir;
    hstep->useDec = Decelerate_USE;
    hstep->buffToUse = 0;
    hstep->Fcur = hstep->Fmin;
    hstep->t = 0.0f;

    if (dir)
        gpio_bits_set(hstep->gpioPort, hstep->gpioPin);
    else
        gpio_bits_reset(hstep->gpioPort, hstep->gpioPin);

    /* 如果没有加速段，直接进入匀速/减速 */
    if (accPulse > 0)
    {
        hstep->state = Acclerate;
        hstep->buffToUse = Step_FillAccelerateFixed(hstep);
    }
    else if (constPulse > 0)
    {
        hstep->state = Constant;
        hstep->Fcur = hstep->Fmax;
        hstep->buffToUse = Step_FillConstantFixed(hstep);
    }
    else
    {
        hstep->state = Decelerate;
        hstep->Fcur = hstep->Fmax;
        hstep->buffToUse = Step_FillDecelerateFixed(hstep);
    }

    return hstep->buffToUse;
}

/**
 * @brief 根据当前状态填充缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量，-1-状态错误
 */
int Step_BuffFill(stepTypedef* hstep)
{
    hstep->buffToUse = 0;

    /* 固定脉冲分段模式下，根据状态填充缓冲区 */
    if (hstep->fixedMode)
    {
        switch (hstep->state)
        {
        case Acclerate:
            hstep->buffToUse = Step_FillAccelerateFixed(hstep);
            break;
        case Constant:
            hstep->buffToUse = Step_FillConstantFixed(hstep);
            break;
        case Decelerate:
            hstep->buffToUse = Step_FillDecelerateFixed(hstep);
            break;
        default:
            return -1;
        }
    }
    else /* Tacc 模式下，根据状态填充缓冲区 */
    {
        switch (hstep->state)
        {
        case Acclerate:
            hstep->buffToUse = Step_FillAccelerate(hstep);
            break;
        case Constant:
            hstep->buffToUse = Step_FillConstant(hstep);
            break;
        case Decelerate:
            hstep->buffToUse = Step_FillDecelerate(hstep);
            break;
        default:
            return -1;
        }
    }
    return hstep->buffToUse;
}
