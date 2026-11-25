# STM32F103 遥控器固件

## 资源链接

- **演示视频**：[YouTube 演示视频](https://youtu.be/B1da01hPyU0?si=EHgXTzQ1f3uk6zeB)
- **硬件开源**：[立创开源广场项目页](https://oshwhub.com/qwqemm/project_teuseijg)

本工程是基于 **STM32F103C8T6** 的遥控器固件。外设初始化由 STM32CubeMX 配置，使用 STM32 HAL 驱动外设、FreeRTOS/CMSIS-RTOS 管理任务，并通过 Keil MDK-ARM 工程编译。

工程包含按键与菜单界面、摇杆 ADC 采样、陀螺仪相关任务、无线通信模块驱动，以及 OLED 显示。具体硬件接线和外设引脚以 `xuexi.ioc` 与 `Core/Src/*_hal_msp.c`、外设初始化源码为准。

## 主要模块

- **输入与控制任务**：`hardware/TASK.c`、`hardware/TASK.h`。工程创建按键、控制、无线通信、摇杆和陀螺仪相关任务，并通过 FreeRTOS 队列、事件组传递数据或同步状态。
- **无线与串口模块**：`hardware/FREQ2_4G.c`、`hardware/FREQ2_4G.h` 实现 SPI 无线模块驱动；`hardware/HLKAs201.c`、`hardware/HLKAs201.h` 实现 HLK-AS201 相关接口。
- **OLED 与菜单**：`hardware/OLED.c`、`hardware/OLED.h` 提供屏幕绘制、状态更新和刷新。页面切换按时间推进并复用上一帧缓存；同级菜单上下选择采用选项滚动。上下滚动和左右页面切换的动画时长、回弹参数分别配置，功能页标题区域保持固定。
- **应用界面状态**：`hardware/fun.c`、`hardware/fun.h` 处理菜单状态与功能页相关操作；`Core/Src/freertos.c` 中的 `defaultTask` 周期调用 `OLED_UIProcess()` 消费界面状态并刷新显示。
- **外设初始化**：`Core/Src` 与 `Core/Inc` 包含 CubeMX 生成或维护的 HAL 外设初始化代码；当前配置包括 ADC、DMA、I2C、SPI、USART、GPIO 和 FreeRTOS。

## 目录结构

```text
.
├── Core/                   # MCU 启动后的应用入口、HAL 外设和 RTOS 配置
│   ├── Inc/                 # 应用及外设头文件
│   └── Src/                 # main、外设初始化、中断和 FreeRTOS 任务
├── Drivers/                 # STM32 HAL 与 CMSIS 驱动
├── Middlewares/             # FreeRTOS 等中间件
├── hardware/                # OLED、按键/控制任务、无线模块等工程模块
├── MDK-ARM/                 # Keil 工程、启动文件和编译输出目录
├── xuexi.ioc                # STM32CubeMX 配置
└── 更新日志.md               # 功能改动、构建结果和验证记录
```

## 编译

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/xuexi.uvprojx`。
2. 确认工程目标为 `xuexi`（器件为 STM32F103C8），然后执行 **Build**。
3. 工程的输出目录配置为 `MDK-ARM/xuexi/`。最近一次记录的完整构建使用 Keil ARMCC 5.06 update 7；实际构建是否成功以本机 Keil 输出为准。

CubeMX 工程配置保存在 `xuexi.ioc`。修改外设配置或重新生成代码前，先核对现有源码中的用户代码区域和工程变更记录，避免覆盖工程逻辑。

## OLED 动画与调参

动画参数集中在 `hardware/OLED.c` 文件开头：

- `OLED_MENU_SCROLL_DURATION_US`：同级菜单选项上下滚动时长。
- `OLED_PAGE_TRANSITION_DURATION_US`：页面进入和退出时长。
- `OLED_MENU_BOUNCE_DURATION_US`、`OLED_MENU_BOUNCE_MAX_PIXELS`：菜单滚动结束时的回弹时长与位移。
- `OLED_PAGE_BOUNCE_DURATION_US`、`OLED_PAGE_BOUNCE_MAX_PIXELS`：页面滑动结束时的回弹时长与位移。

时长使用微秒。菜单滚动与页面滑动有独立参数，可分别调整。动画依据经过时间计算位置；若绘制或传输负载使中间位置过期，会跳过过期帧并继续向最终位置推进。传输失败后使用全屏刷新恢复显示。

OLED 使用 I2C1，当前总线配置为 **400 kHz**。长像素数据可走 DMA；命令和短数据按驱动中的发送策略处理。不要在中断或每帧路径中添加串口打印，以免影响动画时序。

## OLED 耗时统计

调试时可在调试器中查看 `hardware/OLED.h` 声明的全局变量 `OLED_TimingStats`，不需要每帧串口输出。`timer_ready` 为 1 表示 DWT 微秒计时器可用。

| 字段 | 含义 |
| --- | --- |
| `draw_us` | 绘图、动画合成、差异比较和页缓存同步耗时 |
| `i2c_us` | I2C 发送耗时，包含 DMA 完成等待 |
| `frame_us` | 单帧绘制及发送总耗时，不含动画帧间等待 |
| `sent_bytes` | 成功发送的字节数，包含地址、控制字节和载荷 |
| `dma_count` | DMA 发送调用次数 |
| `failure_count` | 本帧 I2C 发送失败次数 |
| `history[]` | 最近 5 帧的统计记录；用 `frame_id` 排序，`history_write_index` 指向下一个写入位置 |

比较动画性能时，分别查看过渡帧的 `draw_us`、`i2c_us` 和 `frame_us`。`frame_us` 不包含动画步骤之间的 RTOS 延时，因此它不等于用户感受到的整段动画时长。

## 程序启动与任务

程序入口位于 `Core/Src/main.c`：初始化 HAL、时钟和外设，然后初始化并启动 RTOS 调度器。`Core/Src/freertos.c` 在调度器启动前初始化 OLED、队列和事件组，并创建各应用任务。`defaultTask` 负责消费并显示最新 UI 状态；其他任务执行按键、控制、无线通信及摇杆/陀螺仪相关工作。

## 变更记录

详细改动、构建结果和验证范围见 [`更新日志.md`](更新日志.md)。源码分析、Keil 编译、离线模型验证和实机烧录/观感验证属于不同验证环节；请以日志中每项记录的验证状态为准。
