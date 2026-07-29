# flux_isolate_wave 系统设计规格书 (System Design Document)

## 1. 概述与技术栈 (Overview & Tech Stack)

* **应用名称**：`flux_isolate_wave` (波浪式芦笋/螺栓单根分离控制系统)
* **主控芯片**：ESP32-D0WD-V3 (Dual-Core 240MHz, 520KB SRAM)
* **开发框架**：PlatformIO + Arduino ESP32 Framework (C++17)
* **操作系统/内核**：FreeRTOS 实时操作系统
* **电机驱动库**：`FastAccelStepper`（基于 ESP32 硬件 Timer/RMT 中断的高性能加减速库）
* **无线通信**：NimBLE-Arduino / ESP32 BLE (Bluetooth Low Energy)
* **扩展芯片**：
  - **74HC595**：8 位串行转并行移位寄存器，用于扩展 8 个步进电机的 DIR 方向线；
  - **74HC165**：8 位并行转串行移位寄存器，用于读取 8 个 Z 轴板的回零限位开关 (HOME)。

---

## 2. 硬件架构与 GPIO 引脚分配

```
                  +-----------------------------------+
                  |           ESP32 MCU               |
                  +-----------------------------------+
                    |   |   |   |   |   |   |   |
     STEP0~7 (Direct GPIO 13,12,14,27,26,25,33,32)
                    |   |   |   |   |   |   |   |
                    v   v   v   v   v   v   v   v
                 +---------------------------------+
                 |  A4988 Stepper Drivers (x8)     |
                 +---------------------------------+
                    ^                           ^
                    | DIR (8 Channels)          | MS1,MS2,MS3
          +-------------------+        (GPIO 4, 16, 17)
          | 74HC595 Shift Reg |
          +-------------------+
            | DS=21, SHCP=19, STCP=18

          +-------------------+
          | 74HC165 Shift Reg | <- 8x HOME Limit Switches
          +-------------------+
            | Q7=34, CLK=19, PL=18
```

---

## 3. 软件架构与组件设计 (Software Architecture)

系统采用面向对象与 FreeRTOS 任务解耦架构，划分为四个核心模块：

### 3.1 `BLEManager` (BLE 通信服务)
* **服务 UUID**：`4fafc201-1fb5-459e-8fcc-c5c9c331914b`
* **Rx Characteristic (Write)**：`beb5483e-36e1-4688-b7f5-ea07361b26a8`
  - 解析来自 Vision 上位机的指令：`COUNT:0`, `COUNT:1`, `COUNT:MANY`, `HOME`, `TEST:M<0-7>`
* **Tx Characteristic (Notify)**：`1c95d5e3-d9d7-4124-a76d-756c0b2100a9`
  - 向上位机推播事件通知：`NOTIFY:READY`, `NOTIFY:EJECT_DONE`, `NOTIFY:STEP_DONE`, `NOTIFY:HOMED`
* 使用 FreeRTOS `QueueHandle_t` 将收到的指令无阻塞传递给 `SortStateMachine`。

### 3.2 `ShiftRegisterManager` (595 / 165 移位寄存器驱动)
* **DIR 端口输出**：`void setMotorDirection(uint8_t motorIdx, bool isClockwise)`
  - 更新内部 8bit DIR 掩码，发送锁存脉冲到 74HC595。
* **HOME 端口输入**：`uint8_t readHomeSwitches()`
  - 脉冲触发 74HC165 `PL` 引脚拉低装载，通过 `CLK` 移位读取 8 路限位开关电平。

### 3.3 `MotorManager` (8轴运动控制层)
* 封装 `FastAccelStepperEngine` 与 8 个 `FastAccelStepper` 实例。
* 提供电机接口：
  - `void init()`：初始化 GPIO、MS 细分引脚及 `FastAccelStepperEngine`。
  - `void setSpeedAndAccel(uint32_t speedHz, uint32_t accel)`：配置加减速参数。
  - `void movePlateTo(uint8_t motorIdx, int32_t targetSteps)`：移动指定板到目标位置。
  - `void lowerPlateToBottom(uint8_t motorIdx)`：将指定挡板降到底部。
  - `bool executeHomingSequence()`：全轴轮询 74HC165 回零归位。

### 3.4 `SortStateMachine` (视觉闭环主状态机)

```
        +---------------+
        |   SYS_INIT    |
        +---------------+
                |
                v
        +---------------+
        |    HOMING     |  <-- 轮询 74HC165 8轴回零
        +---------------+
                | (HOMED -> Notify READY)
                v
        +---------------+
        |     IDLE      |  <-- 等待 BLE 数据 (COUNT:X)
        +---------------+
          /     |     \
COUNT:0  /   COUNT:1   \  COUNT:MANY
        v       v       v
+----------+ +--------+ +---------------------+
| INFEEDA  | | EJECT  | | DE_STACKING_STEP    |
| (Push)   | | SINGLE | | (Lower Plate N)     |
+----------+ +--------+ +---------------------+
        \       |       / (Notify STEP_DONE -> Wait Next COUNT)
         v      v      v
        +---------------+
        |   IDLE STATE  |
        +---------------+
```

---

## 4. 验证与调试计划 (Verification & Testing)

1. **硬件单元验证**：
   - 移位寄存器诊断：测试 74HC595 是否正确刷新 8 路 DIR 引脚，测试 74HC165 是否能准确读取 8 路 HOME 触点。
   - 电机加减速验证：使用 `FastAccelStepper` 测试 8 个 GPIO STEP 脉冲输出，确保电机平稳无丢步。
2. **BLE 通信验证**：
   - 使用手机 BLE 调试助手（如 nRF Connect）连接 ESP32，测试 Notify 接收与 Write 指令响应。
3. **闭环状态机验证**：
   - 模拟 Vision 算法下发 `COUNT:MANY` -> `COUNT:MANY` -> `COUNT:1`，验证离散降板序列与排料通知。
