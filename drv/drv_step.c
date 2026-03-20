/**
 * @file drv_step.c
 * @brief 步进电机脉冲生成驱动程序
 * @details 实现步进电机的梯形/S 曲线速度控制，支持加速、匀速、减速三阶段
 */

#include "drv_step.h"
#include "stdio.h"
#include "math.h"

#ifndef M_PI
#define M_PI (3.1415926535f)  /* 定义圆周率常量 */
#endif

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
    /* 配置定时器和 GPIO */
    hstep->tmr = tmr;
    hstep->channel = channel;
    hstep->gpioPort = GPIOx;
    hstep->gpioPin = GPIO_Pin;

    /* 设置定时器初始值 */
    uint32_t arr = tmr->pr;
    tmr_channel_value_set(tmr, channel, arr / 2);

    /* 保存速度参数 */
    hstep->Fmin = Fmin;
    hstep->Fmax = Fmax;
    hstep->Tacc = Tacc;

    /* 初始化状态变量 */
    hstep->t = -(1000.0f / Fmin);
    hstep->Fcur = Fmin;
    hstep->buffIndex = 0;
    hstep->state = Stop;
    hstep->useDec = 1;

    /* 可选：初始化缓冲区 */
#if AutoInitBuffer
    for(int i=0;i<BufferSize;i++)
    {
        hstep->buff0[i]=0;
        hstep->buff1[i]=0;
    }
#endif
}

/**
 * @brief 填充加速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 * @details 根据梯形曲线或 S 曲线计算加速阶段的频率变化
 */
int Step_FillAccelerate(stepTypedef* hstep)
{
    if (hstep->state != Acclerate)
        return 0;

    /* 计算本次需要填充的脉冲数 */
    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    int i = 0;

    /* 循环计算每个脉冲的定时器值 */
    for (; hstep->t <= hstep->Tacc; i++)
    {
        if (i >= buffToUse)
            break;

        hstep->t += 1000.0f / hstep->Fcur;

        /* 根据曲线类型计算当前频率 */
        switch (AcclerateCurve)
        {
        case Curve_Trapezoidal:  /* 梯形曲线：线性加速 */
            hstep->Fcur = (hstep->Fmax - hstep->Fmin) / hstep->Tacc * hstep->t + hstep->Fmin;
            break;
        case Curve_S:  /* S 曲线：平滑加速 */
            hstep->Fcur = 0.5f * hstep->Fmax * cosf(M_PI - M_PI * hstep->t / hstep->Tacc) + hstep->Fmin + hstep->Fmax * 0.5f;
            break;
        default:
            break;
        }
        /* 计算定时器预分频值 */
        buffPtr[i] = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / hstep->Fcur - 1);
    }

    /* 切换双缓冲区 */
    hstep->buffIndex = !hstep->buffIndex;

    /* 判断是否完成加速阶段 */
    if (i >= buffToUse)
    {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    /* 加速完成，转入匀速阶段 */
    hstep->state = Constant;
    hstep->Fcur = hstep->Fmax;

    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / hstep->Fcur - 1);
    for (; i < buffToUse; i++)
        buffPtr[i] = PSC;

    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 填充匀速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillConstant(stepTypedef* hstep)
{
    if (hstep->state != Constant)
        return 0;

    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    
    /* 计算匀速阶段的定时器预分频值 */
    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / hstep->Fcur - 1);
    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;

    /* 填充缓冲区 */    
    for (int i = 0; i < buffToUse; i++)
        buffPtr[i] = PSC;

    hstep->buffIndex = !hstep->buffIndex;
    hstep->buffRdy = 1;
    return buffToUse;
}

/**
 * @brief 填充减速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillDecelerate(stepTypedef* hstep)
{
    if (hstep->state != Decelerate)
        return 0;

    int buffToUse = hstep->stepToGo > BufferSize ? BufferSize : hstep->stepToGo;
    uint16_t* buffPtr = hstep->buffIndex ? hstep->buff1 : hstep->buff0;
    int i = 0;

		
    /* 循环计算减速阶段的频率变化 */    
    for (; hstep->Fcur > hstep->Fmin; i++)
    {
        if (i >= buffToUse)
            break;

        hstep->t -= 1000.0f / hstep->Fcur;
        hstep->Fcur = (hstep->Fmax - hstep->Fmin) / hstep->Tacc * hstep->t + hstep->Fmin;
        buffPtr[i] = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / hstep->Fcur - 1);
    }

    hstep->buffIndex = !hstep->buffIndex;

    if (i >= buffToUse)
    {
        hstep->buffRdy = 1;
        return buffToUse;
    }

    /* 减速完成，保持最小频率 */    
    hstep->Fcur = hstep->Fmin;
    uint16_t PSC = (uint16_t)((RCC_MAX_FREQUENCY / (hstep->tmr->pr + 1)) / hstep->Fcur - 1);
    for (; i < buffToUse; i++)
        buffPtr[i] = PSC;

    hstep->buffRdy = 1;
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
    hstep->stepToGo -= hstep->buffToUse;

    if (hstep->stepToGo <= 0)
        hstep->state = Stop;
    else if (hstep->stepToGo <= hstep->accStep)
    {
        if (hstep->useDec)
            hstep->state = Decelerate;
        else
            hstep->state = Constant;
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
        hstep->lock = LOCK;
    else
        return -1;
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
        hstep->lock = UNLOCK;
    else
        return -1;
    return 0;
}

/**
 * @brief 中止步进电机运行
 * @param hstep 步进电机控制句柄指针
 */
void Step_Abort(stepTypedef* hstep)
{
    /* 停止定时器 完全停止脉冲，也会停止DMA中断 所以最好再次手动启动中断 */
    tmr_counter_enable(hstep->tmr, FALSE);
    Step_Unlock(hstep);
}

/**
 * @brief 预填充缓冲区，启动步进电机
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
    Step_Lock(hstep);
    hstep->stepToGo = stepToGo;
    hstep->useDec = useDec;

    /* 计算加速阶段所需步数 */    
    switch (AcclerateCurve)
    {
        case Curve_Trapezoidal:
            hstep->accStep = (hstep->Fmin + hstep->Fmax) * hstep->Tacc / 2000 ;
            break;
        case Curve_S:
            hstep->accStep = 0.5f * hstep->Fmax * hstep->Tacc * 0.001f + hstep->Fmin * hstep->Tacc * 0.001f;
            break;
    }
    hstep->buffToUse = 0;

    /* 设置方向 */    
    if (dir)
        gpio_bits_set(hstep->gpioPort, hstep->gpioPin);
    else
        gpio_bits_reset(hstep->gpioPort, hstep->gpioPin);

    /* 根据条件选择初始状态 */    
    if (hstep->Tacc == 0)
    {
        hstep->Fcur = hstep->Fmax;
        hstep->state = Constant;
        hstep->buffToUse = Step_FillConstant(hstep);
    }
    else if (hstep->accStep > stepToGo)
    {
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
 * @brief 根据当前状态填充缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量，-1-状态错误
 */
int Step_BuffFill(stepTypedef* hstep)
{
    hstep->buffToUse = 0;

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
    return hstep->buffToUse;
}
