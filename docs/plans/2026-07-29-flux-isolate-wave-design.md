# flux_isolate_wave 系统设计规格书 (System Design Document)

## 1. 概述与技术栈 (Overview & Tech Stack)

* **应用名称**：`flux_isolate_wave` (波浪式芦笋单根分离控制系统)
* **主控芯片**：ESP32-D0WD-V3 (Dual-Core 240MHz, 520KB SRAM)
* **开发框架**：PlatformIO + Arduino ESP32 Framework (C++17)
* **操作系统/内核**：FreeRTOS 实时操作系统
* **电机驱动库**：`FastAccelStepper`（基于 ESP32 硬件 Timer/RMT 中断的高性能加减速库）
* **无线通信**：NimBLE-Arduino / ESP32 BLE (Bluetooth Low Energy)
* **扩展芯片**：
  - **74HC595**：8 位串行转并行移位寄存器，用于扩展 8 个步进电机的 DIR 方向线；
  - **74HC165**：8 位并行转串行移位寄存器，用于读取 8 个 Z 轴板的回零限位开关 (HOME)。

---

## 2. 物理区域布局 (Physical Layout)

```
 ┌──────────────────────────────────────────────────────────────────────┐
 │                       传送带方向 (芦笋流向) ──────────────────────►  │
 ├──────────┬──────────────────────────────────────────────┬───────────┤
 │          │              挡板区 (Baffle Zone)             │           │
 │ 进料区   │  Z0   Z1   Z2   Z3   Z4   Z5   Z6   Z7      │ 出料区    │
 │ (Infeed) │  ↕    ↕    ↕    ↕    ↕    ↕    ↕    ↕       │ (Outfeed) │
 │  芦笋    │ [板1][板2][板3][板4][板5][板6][板7][板8]     │  单根     │
 │  堆叠    │  ←── 最左/进料端              最右/出料端 ──► │  排出     │
 │  来料    │       ↑                              ↑        │           │
 │          │   HOME限位                        HOME限位    │           │
 ├──────────┴──────────────────────────────────────────────┴───────────┤
 │ 📷 Vision 摄像头                                                      │
 │    监控挡板区，实时发送 BLE 指令: COUNT:0 / COUNT:1 / COUNT:MANY     │
 └──────────────────────────────────────────────────────────────────────┘

 各区域说明:
  • 进料区 (Infeed)  : 芦笋堆叠原料从左侧进入，多根叠放
  • 挡板区 (Baffle)  : 8块Z轴升降板 (Z0~Z7) 依次错落降板，将堆叠芦笋
                       逐步离散分开，使其以「波浪」姿态单根展开
  • 出料区 (Outfeed) : 最右侧板 Z7 动作后，单根芦笋滑入出料槽排出
  • Vision 反馈      : 摄像头俯视挡板区，判断当前区域芦笋根数
                       并通过 BLE 通知 ESP32 执行下一步动作
```

---

## 3. 硬件架构与 GPIO 引脚分配

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
  - 解析来自 Vision 上位机的指令：`COUNT:0`, `COUNT:1`, `COUNT:MANY`, `TEST:M<0-7>`
  - ⚠️ `HOME` 指令**仅在上电初始化时内部自动执行一次**，不作为 BLE 运行时指令处理
* **Tx Characteristic (Notify)**：`1c95d5e3-d9d7-4124-a76d-756c0b2100a9`
  - 向上位机推播事件通知：`NOTIFY:READY`, `NOTIFY:EJECT_DONE`, `NOTIFY:STEP_DONE`, `NOTIFY:HOMED`
* 使用 FreeRTOS `QueueHandle_t` 将收到的指令无阻塞传递给 `SortStateMachine`。
* **BLE 断连策略**：断连不中断当前状态机执行；ESP32 保持运行，重连后 Vision 可直接继续下发指令（无需重新初始化）。

### 3.2 `ShiftRegisterManager` (595 / 165 移位寄存器驱动)
* **DIR 端口输出**：`void setMotorDirection(uint8_t motorIdx, bool isClockwise)`
  - 更新内部 8bit DIR 掩码，发送锁存脉冲到 74HC595。
* **HOME 端口输入**：`uint8_t readHomeSwitches()`
  - 脉冲触发 74HC165 `PL` 引脚拉低装载，通过 `CLK` 移位读取 8 路限位开关电平。
* ⚠️ **共享总线时序约束**：GPIO18（RCLK/PL）与 GPIO19（SRCLK/CLK）被 74HC595 和 74HC165 共用。
  两类操作（写 DIR / 读 HOME）**不可并发**，实现时须以 FreeRTOS `Mutex` 串行化访问，
  且读取 74HC165 期间保持 74HC595 DS 引脚为低电平，防止意外移入脏数据。

### 3.3 `MotorManager` (8轴运动控制层)

**机械参数与步数常量：**

| 参数 | 值 |
|---|---|
| 电机类型 | NEMA 17，200步/转（1.8°/步） |
| 细分设置 | 1/16步（MS1=H, MS2=H, MS3=H） |
| 传动方式 | GT2 同步带，20T 齿轮，40mm/转 |
| 步进分辨率 | **80步/mm** |
| STEPS_HOME（底部，限位触点） | **0步**（绝对零点） |
| STEPS_IDLE（进料等待高度） | **4,000步（+50mm）** |
| STEPS_DESTACK（分离托举高度） | **16,000步（+200mm）** |
| STEPS_TEST_JOG（点动测试步数） | **100步（≈1.25mm）** |
| EJECT_INTERVAL_MS（排出间隔） | **500ms** |
| SPEED_DEFAULT（默认速度） | **800 Hz（步/s）≈ 10mm/s**（现场调试后更新） |
| ACCEL_DEFAULT（默认加速度） | **2000 步/s²**（现场调试后更新） |

* 封装 `FastAccelStepperEngine` 与 8 个 `FastAccelStepper` 实例。
* 提供电机接口：
  - `void init()`：初始化 GPIO、MS 细分引脚及 `FastAccelStepperEngine`。
  - `void setSpeedAndAccel(uint32_t speedHz, uint32_t accel)`：配置加减速参数。
  - `void movePlateTo(uint8_t motorIdx, int32_t targetSteps)`：移动指定板到目标位置。
  - `void lowerPlateToBottom(uint8_t motorIdx)`：将指定挡板降到底部（HOME 位置）。
  - `bool executeHomingSequence()`：全轴轮询 74HC165 回零归位（仅上电初始化时调用）。
  - `void runTestSequence(uint8_t motorIdx)`：**`TEST:M<N>` 测试序列**：
    1. 点动向下约 100 步（验证电机响应）
    2. 继续下降至底部（HOME）
    3. 上升至 +50mm（验证全程往返）


### 3.4 `SortStateMachine` (视觉闭环主状态机)

```
   +-----------+
   |  SYS_INIT |
   +-----------+
         |
         v
   +-----------+
   |  HOMING   |  全轴降到 HOME 限位 → 升至 +50mm → NOTIFY:HOMED
   +-----------+
         |
         v
   +------------------+  <─────────────────────────────────────────┐
   |  INFEED (进料)   |                                             │
   +------------------+                                             │
     1. Z0~Z7 全降至 0  (重力进料)                                  │
     2. Z0~Z7 同步升至 +50mm                                        │
     3. NOTIFY:READY                                                 │
         |                                                           │
         v                                                           │
   +-----------+                                                     │
   |   IDLE    |  等待 Vision BLE 指令 (COUNT:0/1/MANY/HOME)        │
   +-----------+                                                     │
      |       |                                                      │
      |  COUNT:0 (挡板区空)                                          │
      |       └──────────────────────────────────────────────────── ┘
      |                                                              │
      |  COUNT:MANY (多根)                                           │
      └──→ +--------------------+                                    │
           |   DE_STACKING      |                                    │
           | 1. 全升至 +200mm  |                                    │
           | 2. 从 Z0 开始      |                                    │
           | 3. Z[N] 降至 0     |                                    │
           | 4. NOTIFY:STEP_DONE|                                    │
           | 5. 等待 Vision     |                                    │
           |    MANY→N++,回步3  |                                    │
           |    COUNT:0 ─────────────────────────────────────────── ┘
           |    COUNT:1 ─────┐
           +--------------------+   │
                                    v
                         +------------------+
                         |  EJECT_SINGLE    |
                         | 剩余高板(还在200mm)|
                         | 从Z7→Z0依次降至0  |
                         | 每步间隔 500ms    |
                         | NOTIFY:EJECT_DONE |
                         +------------------+
                                  |
                                  └──→ 自动触发 INFEED (回上方循环)
```

**各状态说明：**

| 状态 | 触发条件 | 核心物理动作 | 结束通知 |
|---|---|---|---|
| `HOMING` | 上电初始化 | 8轴降到 HOME 限位 → 升至 +50mm | `NOTIFY:HOMED` |
| `INFEED` | COUNT:0 或 EJECT完成 | 全降至0（重力进料）→ 同步升至 +50mm | `NOTIFY:READY` |
| `DE_STACKING` | COUNT:MANY | 全升至 +200mm → 从Z0起逐板降至0，每降一板等Vision反馈，循环 | `NOTIFY:STEP_DONE`（每板） |
| `EJECT_SINGLE` | COUNT:1 | 剩余高板从Z7→Z0依次降至0，间隔500ms | `NOTIFY:EJECT_DONE` |

---

## 4. 验证与调试计划 (Verification & Testing)

1. **硬件单元验证**：
   - 移位寄存器诊断：测试 74HC595 是否正确刷新 8 路 DIR 引脚，测试 74HC165 是否能准确读取 8 路 HOME 触点。
   - 电机加减速验证：使用 `FastAccelStepper` 测试 8 个 GPIO STEP 脉冲输出，确保电机平稳无丢步。
2. **BLE 通信验证**：
   - 使用手机 BLE 调试助手（如 nRF Connect）连接 ESP32，测试 Notify 接收与 Write 指令响应。
3. **闭环状态机验证**：
   - 模拟 Vision 算法下发 `COUNT:MANY` → `COUNT:MANY` → `COUNT:1`，验证离散降板序列与排料通知。
   - 模拟 `COUNT:MANY` → `COUNT:0`，验证直接触发 INFEED 的路径。

