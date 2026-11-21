# STM32-F103C8T6-KKUI-Controller

基于 **STM32F103C8T6** 的双手柄遥控器固件归档。仓库同时保存左手柄与右手柄两套 Keil MDK-ARM 工程，两套工程共享主要控制、显示和通信实现，并通过角色宏区分左右身份。

## 主要功能

- STM32 HAL 外设初始化与 FreeRTOS/CMSIS-RTOS2 任务框架。
- 左右手柄 USART3 分帧通信，包含握手、心跳、超时检测、序列号和 CRC16 校验。
- 单机 `SOLO`、握手中 `CONNECTING`、双机 `DUAL` 和故障 `FAULT` 链路状态管理。
- 摇杆 ADC 采样、HLK-AS201 姿态数据解析和输入过期保护。
- FREQ2_4G 无线通信及电机/自转控制。
- OLED 菜单、功能页、状态显示和过渡动画。

## 仓库结构

```text
.
├── 左手柄/
│   └── F103/xuexi/          # 左手柄固件工程
├── 右手柄/                  # 右手柄固件工程
└── README.md                # 仓库总说明
```

每套工程内部包含：

- `Core/`：主程序、HAL 外设初始化、中断和 FreeRTOS 配置。
- `Drivers/`：STM32 HAL、CMSIS 及器件支持文件。
- `Middlewares/`：FreeRTOS 中间件。
- `hardware/`：控制器模式、双机链路、OLED、按键、摇杆、姿态与无线模块。
- `MDK-ARM/xuexi.uvprojx`：Keil 工程入口。
- `xuexi.ioc`：STM32CubeMX 配置。
- `更新日志.md`：功能改动、构建结果和验证记录。

## 左右手柄配置

左右角色分别定义在各自工程的 `hardware/controller_config.h`：

```c
#define CONTROLLER_LOCAL_ROLE CONTROLLER_ROLE_LEFT   // 左手柄
#define CONTROLLER_LOCAL_ROLE CONTROLLER_ROLE_RIGHT  // 右手柄
```

双机协议周期、超时、任务优先级、按键定义、CRC 参数和 USART3 配置也集中在该文件中。修改通信参数时必须同步检查左右两套工程。

## 编译

1. 使用 Keil MDK-ARM 打开对应工程的 `MDK-ARM/xuexi.uvprojx`。
2. 确认目标芯片为 STM32F103C8。
3. 分别编译左手柄和右手柄工程。

编译输出目录 `MDK-ARM/xuexi/`、对象文件、日志、缓存和 IDE 本地配置均不纳入版本控制，需要时由本地重新生成。

## 文档

- 左手柄说明：[`左手柄/F103/xuexi/README.md`](左手柄/F103/xuexi/README.md)
- 右手柄说明：[`右手柄/README.md`](右手柄/README.md)
- 左手柄更新日志：[`左手柄/F103/xuexi/更新日志.md`](左手柄/F103/xuexi/更新日志.md)
- 右手柄更新日志：[`右手柄/更新日志.md`](右手柄/更新日志.md)
- RTOS 导读：[`右手柄/RTOS快速导读.md`](右手柄/RTOS快速导读.md)
- 双手柄分工程实施步骤：[`右手柄/双手柄分工程实施步骤.md`](右手柄/双手柄分工程实施步骤.md)

## 归档说明

本仓库用于保存当前完整源码版本。构建产物、日志和本地验证环境不推送；编译、烧录和实机控制效果应以对应更新日志中的验证状态为准。
