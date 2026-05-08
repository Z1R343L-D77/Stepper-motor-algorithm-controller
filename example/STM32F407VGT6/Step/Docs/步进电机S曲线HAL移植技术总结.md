# STM32F407VGT6 HAL 步进电机 S 曲线程序技术总结

## 1. 文档目的

本文档记录本工程中步进电机 S 曲线运动程序的移植方案、硬件资源分配、软件结构、运行流程和后续维护要点。

适用工程：

- MCU：STM32F407VGT6
- HAL 工程：`step/f_cam`
- 编译器：Keil MDK ARMCC V5.06
- 系统主频：168 MHz
- 脉冲输出：TIM1_CH1 / PE9
- 方向输出：PE8

## 2. 硬件资源分配

| 功能 | MCU 引脚 | 外设/模式 | 说明 |
| --- | --- | --- | --- |
| 步进脉冲 PUL | PE9 | TIM1_CH1 / AF1 | 输出脉冲信号 |
| 方向 DIR | PE8 | GPIO Output PP | 输出方向电平 |
| S 曲线 tick | SysTick | 1 ms | 每 1 ms 更新一次速度曲线 |
| 脉冲调度中断 | TIM1_CC_IRQn | Compare 中断 | 每个脉冲边沿进入中断调度 |

当前程序只启用 1 路步进电机通道。

## 3. 时钟与定时器设计

系统主频仍为 168 MHz。

TIM1 位于 APB2，总线配置下 TIM1 定时器时钟为 168 MHz。由于 TIM1 是 16 位定时器，若直接以 168 MHz 计数，低速脉冲周期会超过 16 位计数范围，因此程序将 TIM1 预分频到 1 MHz：

```c
TIM1->PSC = 167U;   // 168 MHz / (167 + 1) = 1 MHz
```

因此程序中的 `STEPPER_TIM_FREQ` / `SYS_FREQ` 表示的是 TIM1 的脉冲调度计数频率，不是 MCU 系统主频。

当前关键配置：

```c
#define STEPPER_TIM_FREQ 1000000UL
#define SYS_FREQ         STEPPER_TIM_FREQ
```

这样设计后：

- 100 kHz 脉冲周期约为 10 个 TIM1 计数。
- 30 Hz 脉冲周期约为 33333 个 TIM1 计数。
- 低速和高速都能落在 16 位 TIM1 的可调度范围内。

## 4. 软件模块结构

新增和修改的核心文件如下：

| 文件 | 作用 |
| --- | --- |
| `Step/Inc/stepper_app.h` | 应用层接口 |
| `Step/Src/stepper_app.c` | 示例运动状态机和电机参数配置 |
| `Step/Inc/stepper_s_curve/stepper_s_curve.h` | S 曲线库统一入口 |
| `Step/Inc/stepper_s_curve/stepper_driver.h` | 步进驱动公共结构体和 API |
| `Step/Src/stepper_s_curve/s_curve_params.c` | S 曲线参数计算 |
| `Step/Src/stepper_s_curve/func_scrv.c` | S 曲线状态机，每 1 ms 更新速度 |
| `Step/Src/stepper_s_curve/stepper_driver.c` | 硬件无关的脉冲与运动控制逻辑 |
| `Step/Src/stepper_s_curve/stepper_driver_stm32f4.c` | STM32F407 HAL 硬件适配层 |
| `Core/Src/main.c` | 调用 `Stepper_AppInit()` 和 `Stepper_AppTask()` |
| `Core/Src/stm32f4xx_it.c` | SysTick 中调用 `Stepper_AppTick1ms()` |
| `MDK-ARM/f_cam.uvprojx` | 已加入新增源文件和 include path |

## 5. 程序启动流程

启动流程如下：

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_TIM1_Init()
  -> Stepper_AppInit()
       -> func_plus_ch_init()
       -> func_plus_init()
            -> __func_plus_msp_init()
                 -> 配置 PE9 为 TIM1_CH1
                 -> 配置 PE8 为方向 GPIO
                 -> 配置 TIM1 比较中断
                 -> 使能 TIM1_CC_IRQn
```

运行中：

```text
SysTick_Handler()
  -> HAL_IncTick()
  -> Stepper_AppTick1ms()
       -> func_plus_tick()
            -> func_scrv_tick()
            -> 更新当前速度和脉冲频率

TIM1_CC_IRQHandler()
  -> func_plus_it()
       -> 翻转/冻结 PE9 输出模式
       -> 更新下一次 CCR1 比较值
       -> 累计当前脉冲位置 plus_cur
```

主循环：

```text
while (1)
  -> Stepper_AppTask()
       -> 根据状态机发出正向/反向 S 曲线定长运动
```

## 6. 当前示例运动逻辑

`stepper_app.c` 中当前示例为循环往返运动：

1. 上电后等待 500 ms。
2. 正向运行 100 mm。
3. 等待运动完成。
4. 暂停 500 ms。
5. 反向运行 100 mm。
6. 等待运动完成。
7. 暂停 500 ms。
8. 回到第 2 步。

关键参数：

```c
#define MOTOR_STEPS_PER_REV   6400.0f
#define MOTOR_LEAD_MM         40.0f
#define MOTOR_PULSE_DIST_MM   (MOTOR_LEAD_MM / MOTOR_STEPS_PER_REV)

#define MOTOR_JERK_MAX        5000.0f
#define MOTOR_ACCEL_MAX       200.0f
#define MOTOR_SPEED_MAX       500.0f

#define DEMO_TRAVEL_MM        100.0f
#define DEMO_CRUISE_SPEED     200.0f
#define DEMO_PAUSE_MS         500U
```

如果实际电机细分、丝杆导程或机械结构不同，优先修改上述参数。

## 7. 核心 API

### 初始化通道

```c
func_plus_ch_init(&g_func_plus.ch[0], &ch_init);
func_plus_init(&g_func_plus);
```

### 定长 S 曲线运动

```c
func_plus_s_L(axis,
              cruise_speed,
              0.0f,
              distance_mm,
              DIR_FORWARD,
              SCRV_CB1);
```

参数说明：

- `cruise_speed`：目标巡航速度，单位 mm/s。
- `0.0f`：末速度为 0，表示到位停止。
- `distance_mm`：运动距离，单位 mm。
- `DIR_FORWARD` / `DIR_INVERSE`：运动方向。
- `SCRV_CB1`：立即执行。

### 查询是否忙碌

```c
func_plus_s_busy(&g_func_plus.ch[0]);
```

返回：

- `1`：正在运动或有待执行参数。
- `0`：空闲。

### 查询剩余脉冲

```c
Stepper_AppRemainingPulses();
```

该函数会在读取 `plus_cur` / `plus_tag` 时临时关闭中断，避免中断更新位置时读到不一致的值。

## 8. 脉冲输出原理

程序没有使用固定 PWM 频率输出，而是使用比较中断动态调度下一次脉冲边沿。

核心逻辑在 `func_plus_it()`：

1. 读取当前 CCR1。
2. 根据当前速度换算出的周期增量 `cmp_incx256` 计算下一次比较值。
3. 通过修改 OC 模式 bit0，在 toggle 和 inactive 之间切换。
4. 在有效脉冲阶段累加或递减 `plus_cur`。

这种方式的优点：

- 可以在运动过程中连续改变脉冲频率。
- 适合 S 曲线加减速。
- 不需要频繁重新初始化 PWM。

## 9. 方向输出逻辑

方向由 PE8 输出。正向对应电平由初始化参数决定：

```c
ch_init.dir_forward = DIR_FORWARD;
```

运动命令中传入：

```c
DIR_FORWARD
DIR_INVERSE
```

驱动层会根据 `dir_forward` 自动换算 PE8 输出电平。

注意：当前速度不为 0 时，程序不允许直接切换相反方向。需要先减速停止，再反向运行。

## 10. 调参与维护建议

### 修改机械参数

修改位置：`Step/Src/stepper_app.c`

```c
#define MOTOR_STEPS_PER_REV
#define MOTOR_LEAD_MM
```

`MOTOR_PULSE_DIST_MM` 必须等于：

```text
每个脉冲移动距离 = 导程 / 每圈脉冲数
```

### 修改速度参数

修改位置：`Step/Src/stepper_app.c`

```c
#define MOTOR_JERK_MAX
#define MOTOR_ACCEL_MAX
#define MOTOR_SPEED_MAX
#define DEMO_CRUISE_SPEED
```

建议调试顺序：

1. 先降低 `DEMO_CRUISE_SPEED`。
2. 再逐步提高 `MOTOR_ACCEL_MAX`。
3. 最后调整 `MOTOR_JERK_MAX`，控制启停柔顺程度。

### 修改运动距离

修改：

```c
#define DEMO_TRAVEL_MM
```

### 修改方向极性

如果实际方向相反，修改：

```c
ch_init.dir_forward = DIR_INVERSE;
```

或交换驱动器 DIR 线极性。

## 11. 重新生成 CubeMX 代码时的注意事项

当前工程尽量把业务代码放在新增文件中，减少 CubeMX 覆盖风险。

如果重新生成 CubeMX，请重点检查：

1. `main.c` 中是否仍包含：

```c
#include "stepper_app.h"
Stepper_AppInit();
Stepper_AppTask();
```

2. `stm32f4xx_it.c` 的 SysTick 中是否仍包含：

```c
Stepper_AppTick1ms();
```

3. Keil 工程是否仍包含新增源文件：

```text
stepper_app.c
s_curve_params.c
func_scrv.c
stepper_driver.c
stepper_driver_stm32f4.c
```

4. Include path 是否包含：

```text
../Step/Inc
../Step/Inc/stepper_s_curve
```

## 12. 编译验证

已使用 Keil ARMCC V5.06 命令行构建通过：

```text
"f_cam\f_cam.axf" - 0 Error(s), 0 Warning(s).
```

生成文件包括：

```text
MDK-ARM/f_cam/f_cam.axf
MDK-ARM/f_cam/f_cam.hex
```

## 13. 当前方案总结

本方案将 demo 中的 S 曲线算法层保留，将原标准外设库硬件层替换为 STM32F407 HAL 工程可用的 TIM1/PE9/PE8 适配层。

关键结论：

- 系统主频保持 168 MHz。
- TIM1 通过 `PSC=167` 变为 1 MHz 计数。
- PE9 输出步进脉冲。
- PE8 输出方向。
- SysTick 每 1 ms 驱动 S 曲线速度更新。
- TIM1_CC 中断负责精确调度脉冲边沿。
- 当前程序已实现可运行的往返 S 曲线定长运动。
