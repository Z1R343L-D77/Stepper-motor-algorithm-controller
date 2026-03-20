/**
 * @file drv_step.h
 * @brief 步进电机脉冲生成驱动程序头文件
 * @details 定义步进电机控制结构体、配置选项和函数接口声明
 */

#ifndef STEPHELPER_H
#define STEPHELPER_H

#include "stdint.h"


/*********************  Config Option Begin  ********************/
#ifndef RCC_MAX_FREQUENCY
#define RCC_MAX_FREQUENCY 240000000         /**< 系统时钟最大频率 (Hz) */
#endif
#define AutoInitBuffer (1)                  /*< 是否自动初始化缓冲区 (0-关闭，1-开启) */
#define AcclerateCurve (Curve_Trapezoidal)  /*< 加速曲线类型 (梯形/S 曲线) */
#define BufferSize (512)                    /*< 双缓冲区大小 (脉冲数量) */    
/*********************  Config Option End  ********************/
#define Decelerate_USE (1U)             /*< 启用减速标志 */
#define Decelerate_NOUSE (0U)           /*< 禁用减速标志 */
#define Curve_Trapezoidal (0x01)        /*< 梯形速度曲线 */
#define Curve_S (0x02)                  /*< S 形速度曲线 */    
/**
 * @brief 步进电机控制结构体
 * @details 包含定时器、GPIO、速度参数、双缓冲区及状态信息
 */
typedef struct stepTypedef
{
    tmr_type *tmr;                    /*< 定时器指针 */
    tmr_channel_select_type channel;  /*< 定时器通道选择 */

    gpio_type *gpioPort;              /*< GPIO 端口指针 */
    uint16_t gpioPin;                 /*< GPIO 引脚编号 */

    float Fmin;                       /*< 最小频率 (Hz) */
    float Fmax;                       /*< 最大频率 (Hz) */
    float Tacc;                       /*< 加速时间 (ms) */

    float t;                          /*< 当前时间 (ms) */
    float Fcur;                       /*< 当前频率 (Hz) */

    uint16_t buff0[BufferSize];       /*< 双缓冲区 0 */
    uint16_t buff1[BufferSize];       /*< 双缓冲区 1 */

    /** @brief 填充状态枚举 */
    enum fillState
    {
        Acclerate,    /*< 加速阶段 */
        Constant,     /*< 匀速阶段 */
        Decelerate,   /*< 减速阶段 */
        Stop          /*< 停止状态 */
    } state;

    /** @brief 运行锁枚举 */
    enum runningLock
    {
        UNLOCK,       /*< 未锁定 */ 
        LOCK          /*< 已锁定 */
    } lock;

    uint32_t stepToGo;                /*< 剩余步数 */
    uint32_t buffToUse;               /*< 缓冲区使用长度 */
    uint32_t accStep;                 /*< 加速阶段所需步数 */

    uint8_t useDec : 1;               /*< 是否使用减速 (1 位域) */
    uint8_t buffIndex : 1;            /*< 当前缓冲区索引 (1 位域) */
    uint8_t buffRdy : 1;              /*< 缓冲区就绪标志 (1 位域) */

    uint8_t flag : 1;                 /*< 通用标志位 (1 位域，易失) */   

} stepTypedef;

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
void Step_Init(stepTypedef *hstep,
               tmr_type *tmr,
               tmr_channel_select_type channel,
               gpio_type *GPIOx,
               uint16_t GPIO_Pin,
               float Fmin,
               float Fmax,
               float Tacc);

/**
 * @brief 缓冲区使用完成回调
 * @param hstep 步进电机控制句柄指针
 * @details 在 DMA 或定时器中断中调用，更新剩余步数和状态
 */
void Step_BufferUsed(stepTypedef *hstep);

/**
 * @brief 检查缓冲区是否就绪
 * @param hstep 步进电机控制句柄指针
 * @return 1-就绪，0-未就绪
 */
int Step_IsBuffRdy(stepTypedef *hstep);

/**
 * @brief 获取当前缓冲区指针
 * @param hstep 步进电机控制句柄指针
 * @return 当前可使用的缓冲区地址
 */
uint16_t *Step_GetCurBuffer(stepTypedef *hstep);

/**
 * @brief 获取缓冲区使用长度
 * @param hstep 步进电机控制句柄指针
 * @return 已填充的脉冲数量
 */
uint32_t Step_BuffUsedLength(stepTypedef *hstep);

/**
 * @brief 锁定步进电机控制
 * @param hstep 步进电机控制句柄指针
 * @return 0-成功，-1-已锁定
 * @details 防止并发修改控制参数
 */
int Step_Lock(stepTypedef *hstep);

/**
 * @brief 解锁步进电机控制
 * @param hstep 步进电机控制句柄指针
 * @return 0-成功，-1-已解锁
 */
int Step_Unlock(stepTypedef *hstep);

/**
 * @brief 中止步进电机运行
 * @param hstep 步进电机控制句柄指针
 * @details 立即停止定时器并解锁
 */
void Step_Abort(stepTypedef *hstep);

/**
 * @brief 预填充缓冲区，启动步进电机
 * @param hstep 步进电机控制句柄指针
 * @param stepToGo 目标步数
 * @param dir 方向 (1-正，0-反)
 * @param useDec 是否使用减速 (1-是，0-否)
 * @return 填充的脉冲数量
 */
int Step_Prefill(stepTypedef *hstep, int stepToGo, uint8_t dir, uint8_t useDec);

/**
 * @brief 根据当前状态填充缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量，-1-状态错误
 * @details 在运行过程中循环调用以持续填充缓冲区
 */
int Step_BuffFill(stepTypedef *hstep);

/**
 * @brief 填充减速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillDecelerate(stepTypedef *hstep);

/**
 * @brief 填充匀速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillConstant(stepTypedef *hstep);

/**
 * @brief 填充加速阶段脉冲缓冲区
 * @param hstep 步进电机控制句柄指针
 * @return 填充的脉冲数量
 */
int Step_FillAccelerate(stepTypedef *hstep);


#endif
