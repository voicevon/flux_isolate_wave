# flux_isolate_wave (波浪式芦笋/螺栓单根分离控制系统) 需求与硬件规格书

## 1. 项目概述与应用场景

* **项目名称**：`flux_isolate_wave`
* **主控平台**：ESP32 微控制器 (C++ / PlatformIO)
* **驱动芯片**：A4988 步进电机驱动模块 (x8)
* **步进电机控制库**：`FastAccelStepper`（基于硬件定时器/中断的高性能多轴加减速电机库）
* **通信方式**：**BLE (Bluetooth Low Energy 蓝牙无线通信)**
* **上位机系统**：Vision 视觉识别系统（通过 BLE 实时发送挡板区物料数量反馈）
* **核心目标**：ESP32 通过 FreeRTOS 多任务架构，基于 BLE 接收视觉反馈（0根、1根、多根），结合 74HC595 (DIR) 和 74HC165 (HOME)，驱动 8 个 Z 轴板进行离散分拣降板与右侧排料，实现堆叠螺栓的分散与**单根单根（Single-item Output）**有序排出。

---

## 2. 详细硬件与 GPIO 引脚定义

### 2.1 步进电机脉冲引脚 (STEP Pins) — ESP32 直连 GPIO

| 电机序号 | 板位置 | 功能 | ESP32 GPIO 引脚 |
| :--- | :--- | :--- | :--- |
| **Motor 0 (Z0)** | 板 1 (最左/进料) | 脉冲 STEP | **GPIO 13** |
| **Motor 1 (Z1)** | 板 2 | 脉冲 STEP | **GPIO 12** |
| **Motor 2 (Z2)** | 板 3 | 脉冲 STEP | **GPIO 14** |
| **Motor 3 (Z3)** | 板 4 | 脉冲 STEP | **GPIO 27** |
| **Motor 4 (Z4)** | 板 5 | 脉冲 STEP | **GPIO 26** |
| **Motor 5 (Z5)** | 板 6 | 脉冲 STEP | **GPIO 25** |
| **Motor 6 (Z6)** | 板 7 | 脉冲 STEP | **GPIO 33** |
| **Motor 7 (Z7)** | 板 8 (最右/出料) | 脉冲 STEP | **GPIO 32** |

### 2.2 电机方向控制 (DIR Pins) — 74HC595 串行扩展

| 信号名称 | 595 引脚 | ESP32 GPIO 引脚 | 功能说明 |
| :--- | :--- | :--- | :--- |
| **SER (DS)** | Pin 14 | **GPIO 21** | 串行数据输入 |
| **SRCLK (SHCP)** | Pin 11 | **GPIO 19** | 移位时钟 |
| **RCLK (STCP)** | Pin 12 | **GPIO 18** | 锁存时钟 |

### 2.3 细分控制 (Microstepping Pins) — ESP32 直连 GPIO

| 细分信号 | ESP32 GPIO 引脚 | 功能说明 |
| :--- | :--- | :--- |
| **MS1** | **GPIO 4** | A4988 细分选择 1 |
| **MS2** | **GPIO 16** | A4988 细分选择 2 |
| **MS3** | **GPIO 17** | A4988 细分选择 3 |

### 2.4 回零/限位开关 (Home / Limit Switches) — 74HC165 串行读取

| 信号名称 | 165 引脚 | ESP32 GPIO 引脚 | 功能说明 |
| :--- | :--- | :--- | :--- |
| **DATAOUT (Q7)** | Pin 9 | **GPIO 34** | 串行数据读取 (Input Only) |
| **CLK (CP)** | Pin 2 | **GPIO 19** | 移位时钟 (与 595 共用) |
| **PL (SH/LD)** | Pin 1 | **GPIO 18** | 并行锁存/装载 (与 595 共用) |

---

## 3. BLE 无线通信与离散闭环状态机

### 3.1 BLE 通信特征 (GATT Service / Characteristics)
* ESP32 广播 BLE 设备服务 `FluxIsolateWave_Service`；
* **Rx Characteristic (上位机 -> ESP32)**：接收 Vision 命令（`COUNT:0`, `COUNT:1`, `COUNT:MANY`, `HOME`）；
* **Tx Characteristic (ESP32 -> 上位机)**：发送状态通知（`NOTIFY:READY`, `NOTIFY:EJECT_DONE`, `NOTIFY:STEP_DONE`）。

### 3.2 软件架构 (C++ / FreeRTOS + FastAccelStepper)
* **FreeRTOS Task 1 (`BLE_Task`)**：运行 ESP32 BLE GATT 服务器，非阻塞监听上位机特征值写入；
* **FreeRTOS Task 2 (`FSM_Task`)**：运行视觉闭环主状态机（`IDLE`, `DE_STACKING_LOWER_PLATE`, `EJECT_SINGLE`, `HOMING`）；
* **FreeRTOS Task 3 (`Motor_Task`)**：使用 `FastAccelStepper` 库基于中断管理 8 个轴的加减速与目标步数，结合 74HC595 动态切换方向，74HC165 定时扫描限位原点。
