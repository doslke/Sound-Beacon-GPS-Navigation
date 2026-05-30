# CYT4BB7 Competition Robot Control System

Embedded firmware for the Infineon CYT4BB7 tri-core MCU, supporting two operational modes: **outdoor GPS waypoint navigation** and **indoor sound-source localization**.

---

## Project Structure

```
lsmall/
├── outdoor/    # Outdoor GPS waypoint navigation
└── indoor/     # Indoor sound-source localization
```

Both variants share the same hardware abstraction layer (Seekfree CYT4BB open-source library) and differ only in application logic.

---

## Hardware Platform

| Item | Specification |
|------|---------------|
| MCU | Infineon CYT4BB7CEA (3 cores: 2× Cortex-M7 @ 250 MHz + 1× Cortex-M0+) |
| IDE | IAR Embedded Workbench for ARM 9.40.1 |
| SDK | Infineon PDL (Peripheral Driver Library) |
| DSP | ARM CMSIS-DSP (FFT, complex math, fast square root) |
| License | GPL v3.0 |

---

## Multi-Core Architecture

```
CM0+  ──boot──▶  CM7-0  ──ADC sampling──▶  shared memory
                                                 │
                 CM7-1  ◀──read data────────────┘
                   │
                   └──▶  servo / motor PWM output
```

- **CM0+**: Boots first, enables both CM7 cores, then idles.
- **CM7-0**: Dedicated high-speed ADC sampling. Continuously reads 4 microphone channels into shared buffers and writes buffer addresses to flash for CM7-1 to locate after reset.
- **CM7-1**: Main control loop — reads sensor data, computes direction, drives servo and motor PWM.

Inter-core data sharing uses `SCB_CleanInvalidateDCache()` (D-cache flush) and flash-stored pointer addresses.

---

## Outdoor Mode — GPS Waypoint Navigation

### Workflow

1. **Waypoint collection**: Press KEY3 to record the current GPS coordinate (averaged over 50 samples); press KEY4 to finish.
2. **Autonomous navigation**: The vehicle drives toward each waypoint in sequence. It advances to the next waypoint when the distance drops to ≤ 2 m or the bearing deviation exceeds 90°, and stops after all waypoints are visited.

### Core Algorithms

**Servo PD Controller** (`PID.c`)

```
target  = get_two_points_azimuth(current GPS position, next waypoint)
feedback = gnss.direction  (GPS course-over-ground)
error   = target - feedback  (handles 360°/0° wraparound)
output  = mid - (Kp × error + Kd × Δerror)
output range: [520, 1020] PWM counts (50 Hz servo)
```

**IMU Attitude Estimation** (`Gyroscope_solve.c`, optional)

- Mahony complementary filter: quaternion integration with PI correction from accelerometer cross-product error.
- 1D Kalman filter: fuses accelerometer and gyroscope for roll angle estimation.

### Peripherals

| Peripheral | Purpose |
|------------|---------|
| UART_2 (P10_0/P10_1) | GNSS module (TAU1201 GPS receiver) |
| UART_1 (P04_0/P04_1) | Wireless module / external number receiver |
| UART_4 | RC receiver (SBUS / custom protocol) |
| TCPWM_CH11 (P01_1) | Steering servo (50 Hz) |
| TCPWM_CH25 (P09_1) | Drive motor (1000 Hz) |
| GPIO P20_0–P20_3 | Push buttons KEY1–KEY4 |
| IPS114 SPI display | Debug output |
| PIT_CH0 | 10 ms timer → `ServoPID()` interrupt |
| Flash | Persistent waypoint coordinate storage |

---

## Indoor Mode — Sound-Source Localization

### Workflow

1. CM7-0 continuously samples 4 microphone channels (2048-sample buffer per channel).
2. CM7-1 runs GCC-SCOT cross-correlation on microphone pairs to compute time-difference of arrival (TDOA).
3. Four TDOA values are converted to angles and averaged to produce the sound direction `dir_num` (±180°).
4. The servo output is computed from `dir_num`; when `|dir_num| > 100°`, reverse-gear logic is triggered.

### Core Algorithms

**GCC-SCOT Sound Localization** (`voice.c`)

```
For each microphone pair:
  1. Compute 2048-point FFT on both signals (CMSIS arm_cfft_f32)
  2. Compute cross-power spectrum, apply SCOT whitening normalization
  3. IFFT to obtain the generalized cross-correlation function
  4. Peak index = TDOA (sample lag)
  5. Convert lag to angle via atan2

Final direction = mean of 4 angle estimates
                  (outliers rejected by norightangle() if any two differ > 90°)
```

**Servo Output**

```
out = 740 - kp × dir_num
|dir_num| > 100°: GPIO P09_0 set high → reverse relay engaged
```

### Peripherals

| Peripheral | Purpose |
|------------|---------|
| ADC1_CH25, ADC1_CH00, ADC1_CH09, ADC0_CH01 | 4-channel microphone sampling |
| TCPWM_CH11 (P01_1) | Steering servo (50 Hz) |
| TCPWM_CH25 (P09_1) | Drive motor (1000 Hz) |
| GPIO P09_0 | Reverse direction relay |
| PIT_CH0 | 10 ms timer → servo control interrupt |
| PIT_CH2 | 100 ms timer → UART number polling |
| Flash | ADC buffer address persistence |

---

## Fast Math Library (`mymath.c`)

| Function | Description |
|----------|-------------|
| `myRsqrt()` | Quake III fast inverse square root (magic constant `0x5f375a86`) |
| `fast_atan2()` | 256-entry lookup table with linear interpolation |
| `arcsin_lookup()` | 1000-entry precomputed arcsin table |
| `KalmanFilter()` | Generic 1D Kalman filter |

---

## Build & Flash

1. Open the IAR Embedded Workbench workspace (`.eww`) for the target variant:
   - Outdoor: `outdoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`
   - Indoor: `indoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`
2. Build all three core configurations: CM0+, CM7-0, CM7-1.
3. Flash each core's binary via J-Link or the on-board debugger.
4. On power-up, CM0+ automatically starts the other two cores.

---

## Key Source Files

| File | Description |
|------|-------------|
| `project/user/main_cm0plus.c` | CM0+ boot core, launches CM7 cores |
| `project/user/main_cm7_0.c` | CM7-0: ADC sampling main loop |
| `project/user/main_cm7_1.c` | CM7-1: main control loop |
| `project/user/cm7_1_isr.c` | CM7-1 interrupt service routines |
| `project/code/GPST.c` | GPS waypoint collection and tracking |
| `project/code/PID.c` | Servo PD controller |
| `project/code/voice.c` | GCC-SCOT sound-source localization |
| `project/code/Gyroscope_solve.c` | Mahony + Kalman IMU attitude solver |
| `project/code/mymath.c` | Fast math utilities |
| `project/code/init.c` | Hardware peripheral initialization |

---

## License

This project is open-source under [GPL v3.0](outdoor/LICENSE). The underlying library is provided by Seekfree Technology (逐飞科技) as part of the CYT4BB open-source library.
