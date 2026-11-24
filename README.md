# My_FOC_v1_code

### STM32F407-Based PMSM Field-Oriented Control

基于 **STM32F407IGT6** 的永磁同步电机（PMSM）磁场定向控制（FOC）学习与开发项目。

本项目使用 STM32CubeMX 配置外设，采用 C 语言和 CMake 组织工程，计划逐步实现电角度校准、Clarke/Park 坐标变换、SVPWM、电流闭环及速度闭环控制。

项目以理解控制原理、独立设计与实际硬件验证为目标。

> **Project Status: Under Development**
>
> 本项目目前处于开发阶段，尚未完成完整 FOC 闭环控制及实物验证。代码和控制参数会随开发过程持续更新。

---

## 1. Hardware Platform

| Item | Description |
|---|---|
| Microcontroller | STM32F407IGT6 |
| Motor Type | PMSM / BLDC |
| Development Tools | STM32CubeMX |
| Programming Language | C |
| Build System | CMake |
| Rotor Position Sensor | AS5600（计划使用） |
| Motor Control Method | Field-Oriented Control |

配套设计中的三相驱动板：

[My_FOC_v1_Driver — DRV8313 Motor Driver PCB](https://github.com/DearDre4m/My_FOC_v1_Driver)

注：硬件驱动板目前为 V1 设计版本，尚未完成实物验证，软件与硬件的完整适配仍需测试。

---

## 2. Development Progress

状态说明：

- ✅ Completed — 已完成
- 🚧 In Progress — 正在开发
- ⬜ Planned — 计划实现

| Module | Status | Description |
|---|---|---|
| STM32CubeMX Project | ✅ | 已建立 STM32F407 基础工程 |
| CMake Build Configuration | ✅ | 已建立 CMake 工程配置 |
| HAL / CMSIS Integration | ✅ | 已生成并纳入工程 |
| TIM / PWM Configuration | 🚧 | 定时器及三相 PWM 参数配置 |
| AS5600 Interface | 🚧 | 位置传感器通信与角度获取 |
| Electrical Angle Calibration | 🚧 | 转子电角度零点校准 |
| Clarke Transformation | ⬜ | 三相电流转换至 αβ 坐标系 |
| Park Transformation | ⬜ | αβ 坐标系转换至 dq 坐标系 |
| SVPWM Integration | ⬜ | 空间矢量 PWM 控制实现与验证 |
| ADC Current Sampling | ⬜ | 三相电流采样与偏置校准 |
| Current PI Control | ⬜ | Id / Iq 双电流闭环 |
| Speed PI Control | ⬜ | 电机转速闭环控制 |
| Protection & Fault Handling | ⬜ | 过流、故障检测及安全停机 |
| Hardware Validation | ⬜ | 实物调试与闭环验证 |

以上为项目开发进度记录。状态应随实际源码实现和测试结果更新。

---

## 3. Planned FOC Architecture

计划采用典型的 PMSM 矢量控制结构：

```text
            Speed Reference
                   |
                   v
             Speed PI
                   |
                   v
               Iq_ref
                   |
         +---------+---------+
         |                   |
       Id_ref              Iq_ref
         |                   |
         v                   v
       Id PI               Iq PI
         |                   |
         +---------+---------+
                   |
                   v
             Inverse Park
                   |
                   v
                SVPWM
                   |
                   v
              PWM Output
                   |
                   v
             Motor Driver
                   |
                   v
                  PMSM

Feedback:
  Phase Current -> ADC -> Clarke -> Park

  AS5600 -> Electrical Angle -> Park / Inverse Park
```

该架构表示计划实现的控制系统，不代表当前代码已经完成全部模块。

---

## 4. Project Structure

```text
My_FOC_v1_code/
|
|-- Core/
|   |-- Inc/
|   `-- Src/
|
|-- Drivers/
|   |-- CMSIS/
|   `-- STM32F4xx_HAL_Driver/
|
|-- cmake/
|
|-- My_FOC.ioc
|-- CMakeLists.txt
|-- CMakePresets.json
|-- startup_stm32f407xx.s
|-- STM32F407XX_FLASH.ld
|-- .gitignore
`-- README.md
```

具体文件结构以仓库最新提交为准。

---

## 5. Development Roadmap

下一阶段计划按照以下顺序推进：

1. 完成 PWM 外设配置与基础输出测试。
2. 完成 AS5600 角度读取及电角度零点校准。
3. 集成 Clarke、Park 变换与 SVPWM。
4. 完成 ADC 电流采样与零电流偏置校准。
5. 实现 Id / Iq 电流 PI 控制。
6. 实现速度 PI 闭环控制。
7. 完成故障保护和硬件调试。

---

## 6. Related Repository

**Hardware — Three-Phase Motor Driver**

[My_FOC_v1_Driver](https://github.com/DearDre4m/My_FOC_v1_Driver)

该仓库包含 DRV8313 三相驱动板的 Altium Designer 原理图、PCB、元器件封装库和 Gerber 制造文件。

---

## 7. Disclaimer

This project is intended for educational and experimental purposes.

The firmware is under active development and has not been fully validated on physical hardware.

Please verify hardware compatibility, motor parameters, current limits and protection mechanisms before operating a motor.

---

**Designed and developed by [DearDre4m](https://github.com/DearDre4m)**
