# RTOS 快速导读

这份导读只讲当前工程里实际用到的 FreeRTOS：系统如何启动、任务如何协作，以及按键和控制数据怎样流动。先看下面的总图，再按“从哪里读”逐段打开源码。

## 先记住这条数据流

```text
main 启动调度器
  ├─ KEY 外部中断 ──事件组──> KEYTASK ──更新菜单状态──> CTRLTASK
  ├─ ADC + DMA ─────────────> JSTask ──最新摇杆值队列──┐
  ├─ USART1 姿态数据 ───────> GYROTask ─最新姿态值队列─┤
  │                                                    v
  │                                  CTRLTASK / fun.c 生成控制命令
  │                                                    │
  │                                                    v
  │                                   单槽控制队列──> FREQ2_4GTask──SPI
  └─ defaultTask <──提交的 UI 状态── OLED 绘制与刷新
```

## 1. 系统从哪里启动

`main()` 先初始化 FreeRTOS 内核，再创建工程里的 RTOS 对象和任务，最后启动调度器：[`Core/Src/main.c`](Core/Src/main.c:112)。调用 `osKernelStart()` 后，任务开始运行；正常情况下不会再返回到 `main()` 后面的普通流程。

对象和任务集中在 [`Core/Src/freertos.c`](Core/Src/freertos.c:97)：

- 调度器启动前调用 `OLED_Init()`（97–103 行）。
- 创建按键锁定软件定时器（117–127 行）。
- 创建摇杆、姿态、无线控制三个长度为 1 的队列，以及按键事件组（137–153 行）。
- 创建 OLED `defaultTask` 和 KEY、CTRL、无线、摇杆、姿态五个任务（158–176 行）。

## 2. RTOS 概念在本工程里的对应物

| 概念 | 在这里做什么 | 代码位置 |
|---|---|---|
| 任务与调度 | 把按键、控制、采样、无线发送和 OLED 刷新拆成独立执行流程 | [`freertos.c`](Core/Src/freertos.c:158)、[`TASK.c`](hardware/TASK.c:50) |
| 阻塞与延时 | 等队列、事件或延时到期时，任务让出 CPU，调度器可运行其他就绪任务 | [`TASK.c`](hardware/TASK.c:62)、[`fun.c`](hardware/fun.c:80) |
| 队列 | 按值传递摇杆、姿态和控制命令；当前长度为 1 的队列保存最新值 | [`freertos.c`](Core/Src/freertos.c:137)、[`TASK.c`](hardware/TASK.c:456) |
| 事件组 | 中断只通知“有按键要处理”，KEYTASK 醒来后再读取各按键 GPIO | [`fun.c`](hardware/fun.c:679)、[`TASK.c`](hardware/TASK.c:62) |
| 软件定时器 | 按键锁定定时结束后由回调恢复状态，不用单独占用一个循环任务 | [`freertos.c`](Core/Src/freertos.c:117) |
| 临界区 | 让多个菜单字段作为一个一致快照读取，避免读到一半新、一半旧的状态 | [`TASK.c`](hardware/TASK.c:19) |
| Tick 与周期任务 | 用系统节拍做超时和周期调度；本工程 Tick 配置为 1 kHz | [`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:64) |

### 任务什么时候运行？

`configUSE_PREEMPTION` 为 1，表示启用抢占式调度（[`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:64)）。任务遇到 `vTaskDelayUntil()`、等待队列或等待事件组时会进入阻塞状态；等到时间到或数据/事件到达，才重新变为就绪态。任务阻塞期间不需要靠空转“等”数据。

系统 Tick 为 1000 Hz，所以一个 tick 名义上是 1 ms（[`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:69)）。`vTaskDelay(5)` 是延时 5 个 tick；`vTaskDelayUntil()` 则按上一次计划唤醒时刻推进，适合稳定采样周期。它们都受系统调度和函数执行时间影响，不等于硬件级精确计时。

## 3. 按键：中断通知，任务处理

1. 按键 EXTI 回调调用 `xEventGroupSetBitsFromISR()` 设置同一个事件位（[`fun.c`](hardware/fun.c:679)）。中断里只发通知，不在中断里更新菜单或画屏。
2. `KEYTASK` 在 `xEventGroupWaitBits()` 上等待；事件位到达后，清位并读取 GPIO 当前电平（[`TASK.c`](hardware/TASK.c:60)）。
3. KEYTASK 更新菜单字段，并用短临界区保护需要一起变化的状态；CTRLTASK 和 UI 读取菜单快照时也通过 `TASK_GetMenuSnapshot()` 保持字段一致（[`TASK.c`](hardware/TASK.c:19)）。

事件组的位是“标志”，不是计数器。短时间多次按键中断可能合并成一次通知，所以 KEYTASK 被唤醒后检查的是当前按键电平，而不是逐个重放每次边沿。

中断里调用 FreeRTOS 的 `...FromISR` API 时，NVIC 优先级必须满足 FreeRTOS 的系统调用优先级限制。本工程 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 为 5，EXTI15_10 也配置为 5（[`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:132)、[`gpio.c`](Core/Src/gpio.c:101)）；以后若调整中断优先级，要同时核对这条约束。

## 4. 摇杆和姿态：生产者只发布最新样本

- `JSTask` 启动 ADC DMA，约每 5 个 tick 取样并累计十次，随后把摇杆值和采样时刻写进单槽队列（[`TASK.c`](hardware/TASK.c:419)、[`Mydef.h`](hardware/Mydef.h:44)）。
- `GYROTask` 每轮先读取姿态数据，再等待 5 个 1-tick 间隔后发布到单槽队列（[`TASK.c`](hardware/TASK.c:463)、[`Mydef.h`](hardware/Mydef.h:74)）。**注意当前实际执行：** 传感器读取在 476–478 行，只发生在累加循环之前；480–485 行的 5 次累加重复使用同一组已保存的姿态值，因此不是 5 次独立传感器采样的平均。看代码时以执行位置为准，不要只按注释推断。
- 两个采样队列都用 `xQueueOverwrite()`，队列长度为 1（[`freertos.c`](Core/Src/freertos.c:139)）。这是“最新值邮箱”：消费者来晚时旧样本会被新样本覆盖，适合摇杆/姿态这种持续变化的数据，不适合必须逐条保存的事件记录。

队列发送的是结构体副本，不是指向任务局部变量的指针；`SampleTick` 用于判断数据是否太旧。

## 5. 控制与超时保护

`CTRLTASK` 根据菜单快照选择当前控制模式（[`TASK.c`](hardware/TASK.c:226)），具体的摇杆/姿态换算在 `fun.c`。例如 `SP_JS_ON()` 等待摇杆队列数据最多 100 ms，并检查样本年龄；超时或数据过期时，清零对应速度字段并覆盖无线控制队列，形成安全停止命令（[`fun.c`](hardware/fun.c:3)、[`fun.c`](hardware/fun.c:51)、[`fun.c`](hardware/fun.c:80)）。姿态模式也有对应的输入超时处理。

注意：CTRLTASK 开头虽然使用 5 ms 的 `vTaskDelayUntil()`（[`TASK.c`](hardware/TASK.c:231)），但启用的控制分支可能在 `xQueueReceive()` 等待新输入，最长阻塞 100 ms。因此不能把整个控制路径理解成“每 5 ms 必定完成一次”。

## 6. 无线发送与 OLED

- `FREQ2_4GTask` 等待控制队列中的最新命令，再组帧并通过 SPI 发送（[`TASK.c`](hardware/TASK.c:348)）。队列同样只有一个位置，新的控制状态可以覆盖尚未处理的旧状态。
- `defaultTask` 独立调用 `OLED_UIProcess()` 消费 UI 状态，并每 10 tick 让出 CPU；它还每秒更新一次任务栈余量统计（[`freertos.c`](Core/Src/freertos.c:195)）。因此按键/控制任务提交界面状态与 OLED 实际绘制是分开的。

## 7. 内存和故障观察

任务和队列由动态分配创建；FreeRTOS 堆配置为 10240 字节，动态分配和静态分配均启用（[`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:64)）。创建失败会走 `Error_Handler()`（[`freertos.c`](Core/Src/freertos.c:147)）。栈溢出和堆分配失败钩子已启用（[`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:49)）。

调试器中可以观察 `gOLEDTaskStackHighWaterWords`、`gKEYTaskStackHighWaterWords`、`gCTRLTaskStackHighWaterWords`、`gFREQTaskStackHighWaterWords`、`gJSTaskStackHighWaterWords`、`gGYROTaskStackHighWaterWords`（[`freertos.c`](Core/Src/freertos.c:49)）。它们表示任务历史运行中最低的剩余栈空间，单位是 `StackType_t` 字数，不是字节；越接近 0 越需要关注。实际余量要在目标板运行后读取。

## 建议的源码阅读顺序

1. [`main.c`](Core/Src/main.c:112)：确认内核初始化和调度器启动点。
2. [`freertos.c`](Core/Src/freertos.c:117)：看对象创建、队列长度、任务入口和栈监测。
3. [`TASK.c`](hardware/TASK.c:50)：先读 KEYTASK，再读 CTRLTASK，然后看三个数据生产/发送任务。
4. [`fun.c`](hardware/fun.c:51)：从安全停止、一个输入超时分支读懂“队列数据如何变成控制命令”。
5. [`FreeRTOSConfig.h`](Core/Inc/FreeRTOSConfig.h:64)：回头核对 Tick、堆、抢占和诊断开关。

## 10 分钟调试器练习

在安全台架或不驱动执行机构的条件下，运行程序并观察：

1. 监视 `gCTRLTaskStackHighWaterWords` 和 `gJSTaskStackHighWaterWords`，等待至少 1 秒，看它们是否更新。
2. 在 `JSTask` 的 `xQueueOverwrite()`（`TASK.c` 第 456 行）和 `SP_JS_ON()` 的 `xQueueReceive()`（`fun.c` 第 80 行）设断点，观察生产者写入后消费者何时取走，以及 `SampleTick` 如何随样本更新。
3. 在调试器中看 KEYTASK 等待事件组时处于阻塞状态；触发按键后，确认它醒来再读 GPIO。不要在 ISR 里单步停留太久，以免扰乱时序。

这组观察能把“任务阻塞/就绪、事件通知、单槽队列覆盖和栈水位”与实际运行对应起来。源码阅读只能说明设计路径，调试器观察结果才是这块板子的运行证据。
