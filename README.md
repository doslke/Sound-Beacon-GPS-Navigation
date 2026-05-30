# CYT4BB7 竞赛小车控制系统

基于 Infineon CYT4BB7 双核 MCU 的竞赛机器人控制固件，支持两种运行模式：**室外 GPS 循迹**与**室内声源定位导航**。

---

## 项目结构

```
lsmall/
├── outdoor/    # 室外 GPS 航点导航
└── indoor/     # 室内声源定位导航
```

两个子项目共享同一套硬件抽象层（Seekfree CYT4BB 开源库），仅应用层逻辑不同。

---

## 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | Infineon CYT4BB7CEA（三核：2× Cortex-M7 @ 250 MHz + 1× Cortex-M0+） |
| IDE | IAR Embedded Workbench for ARM 9.40.1 |
| SDK | Infineon PDL（外设驱动库） |
| DSP | ARM CMSIS-DSP（FFT、复数运算、快速平方根） |
| 许可证 | GPL v3.0 |

---

## 多核架构

```
CM0+  ──启动──▶  CM7-0  ──ADC采样──▶  共享内存
                                          │
                CM7-1  ◀──读取数据────────┘
                  │
                  └──▶  舵机 / 电机 PWM 输出
```

- **CM0+**：上电后启动两个 CM7 核心，随后进入空闲。
- **CM7-0**：专用高速 ADC 采样，持续读取 4 路麦克风数据到共享缓冲区，并将缓冲区地址写入 Flash 供 CM7-1 读取。
- **CM7-1**：主控制循环，读取传感器数据、计算方向、输出舵机与电机 PWM。

核间数据共享通过 `SCB_CleanInvalidateDCache()`（D-Cache 刷新）和 Flash 存储指针地址实现。

---

## 室外模式（GPS 航点导航）

### 工作流程

1. **航点采集阶段**：按下 KEY3 记录当前 GPS 坐标（50 次采样取均值），按下 KEY4 结束采集。
2. **自动导航阶段**：小车依次驶向各航点，到达后（距离 ≤ 2 m 或方位偏差 > 90°）自动切换到下一个，全部完成后停车。

### 核心算法

**舵机 PD 控制器**（`PID.c`）

```
目标角 = get_two_points_azimuth(当前GPS位置, 下一航点)
反馈角 = gnss.direction（GPS 航向角）
误差   = 目标角 - 反馈角（处理 360°/0° 跨越）
输出   = mid - (Kp × error + Kd × Δerror)
输出范围：[520, 1020] PWM 计数（50 Hz 舵机）
```

**IMU 姿态解算**（`Gyroscope_solve.c`，可选）

- Mahony 互补滤波器：四元数积分 + 加速度计叉积误差 PI 修正。
- 1D 卡尔曼滤波：融合加速度计与陀螺仪的横滚角估计。

### 使用的外设

| 外设 | 用途 |
|------|------|
| UART_2（P10_0/P10_1） | GNSS 模块（TAU1201 GPS 接收机） |
| UART_1（P04_0/P04_1） | 无线模块 / 外部数字接收 |
| UART_4 | 遥控接收机（SBUS/自定义协议） |
| TCPWM_CH11（P01_1） | 转向舵机（50 Hz） |
| TCPWM_CH25（P09_1） | 驱动电机（1000 Hz） |
| GPIO P20_0–P20_3 | 按键 KEY1–KEY4 |
| IPS114 SPI 屏幕 | 调试信息显示 |
| PIT_CH0 | 10 ms 定时器 → `ServoPID()` 中断 |
| Flash | 航点坐标持久化存储 |

---

## 室内模式（声源定位导航）

### 工作流程

1. CM7-0 持续采集 4 路麦克风信号（每路 2048 点缓冲区）。
2. CM7-1 对麦克风信号对进行 GCC-SCOT 互相关运算，得到到达时间差（TDOA）。
3. 将 4 组 TDOA 转换为角度，取均值得到声源方向 `dir_num`（±180°）。
4. 根据方向角输出舵机控制量；当 `|dir_num| > 100°` 时触发倒车逻辑。

### 核心算法

**GCC-SCOT 声源定位**（`voice.c`）

```
对每对麦克风：
  1. 对两路信号分别做 2048 点 FFT（CMSIS arm_cfft_f32）
  2. 计算互功率谱，SCOT 加权归一化
  3. IFFT 得到广义互相关函数
  4. 取峰值位置作为 TDOA（样本延迟）
  5. 通过 atan2 将延迟转换为角度

最终方向 = 4 组角度估计的均值（异常值通过 norightangle() 剔除）
```

**舵机输出**

```
out = 740 - kp × dir_num
|dir_num| > 100° 时：GPIO P09_0 置高，触发倒车继电器
```

### 使用的外设

| 外设 | 用途 |
|------|------|
| ADC1_CH25, ADC1_CH00, ADC1_CH09, ADC0_CH01 | 4 路麦克风采样 |
| TCPWM_CH11（P01_1） | 转向舵机（50 Hz） |
| TCPWM_CH25（P09_1） | 驱动电机（1000 Hz） |
| GPIO P09_0 | 倒车方向继电器 |
| PIT_CH0 | 10 ms 定时器 → 舵机控制中断 |
| PIT_CH2 | 100 ms 定时器 → UART 数字轮询 |
| Flash | ADC 缓冲区地址持久化 |

---

## 快速数学库（`mymath.c`）

| 函数 | 说明 |
|------|------|
| `myRsqrt()` | Quake III 快速逆平方根（魔数 `0x5f375a86`） |
| `fast_atan2()` | 256 项查找表 + 线性插值 |
| `arcsin_lookup()` | 1000 项预计算 arcsin 表 |
| `KalmanFilter()` | 通用 1D 卡尔曼滤波器 |

---

## 编译与烧录

1. 使用 **IAR Embedded Workbench for ARM 9.40.1** 打开对应子项目下的 `.eww` 工作区文件：
   - 室外：`outdoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`
   - 室内：`indoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`
2. 选择目标配置（CM0+、CM7-0、CM7-1），依次编译。
3. 通过 J-Link 或板载调试器烧录三个核心的固件。
4. 上电后 CM0+ 自动启动其余两个核心。

---

## 关键源文件索引

| 文件 | 说明 |
|------|------|
| `project/user/main_cm0plus.c` | CM0+ 启动核，引导 CM7 |
| `project/user/main_cm7_0.c` | CM7-0：ADC 采样主循环 |
| `project/user/main_cm7_1.c` | CM7-1：主控制循环 |
| `project/user/cm7_1_isr.c` | CM7-1 中断服务程序 |
| `project/code/GPST.c` | GPS 航点采集与跟踪 |
| `project/code/PID.c` | 舵机 PD 控制器 |
| `project/code/voice.c` | GCC-SCOT 声源定位 |
| `project/code/Gyroscope_solve.c` | Mahony + 卡尔曼 IMU 解算 |
| `project/code/mymath.c` | 快速数学工具函数 |
| `project/code/init.c` | 硬件外设初始化 |

---

## 许可证

本项目基于 [GPL v3.0](outdoor/LICENSE) 开源，底层库来自逐飞科技（Seekfree Technology）CYT4BB 开源库。
