# CYT4BB7 Competition Car Control System

A competition robot control firmware based on the **Infineon CYT4BB7 dual-core MCU**, supporting two operating modes:

* **Outdoor GPS Waypoint Navigation**
* **Indoor Acoustic Source Localization and Navigation**

---

## Project Structure

```text
lsmall/
├── outdoor/    # Outdoor GPS waypoint navigation
└── indoor/     # Indoor acoustic source localization and navigation
```

Both subprojects share the same hardware abstraction layer based on the **Seekfree CYT4BB open-source library**. The application-layer logic differs between the two modes.

---

## Hardware Platform

| Component   | Specification                                                                       |
| ----------- | ----------------------------------------------------------------------------------- |
| **MCU**     | Infineon CYT4BB7CEA — 3-core architecture: 2 × Cortex-M7 @ 250 MHz + 1 × Cortex-M0+ |
| **IDE**     | IAR Embedded Workbench for ARM 9.40.1                                               |
| **SDK**     | Infineon PDL (Peripheral Driver Library)                                            |
| **DSP**     | ARM CMSIS-DSP — FFT, complex-number operations, fast inverse square root            |
| **License** | GPL v3.0                                                                            |

---

## Multicore Architecture

```text
CM0+ ── Boot ──▶ CM7-0 ── ADC Sampling ──▶ Shared Memory
                                           │
                    CM7-1 ◀── Read Data ──┘
                      │
                      └──▶ Servo / Motor PWM Output
```

### Core Responsibilities

* **CM0+**

  * Boots and starts both CM7 cores after power-on.
  * Remains idle after initialization.

* **CM7-0**

  * Performs high-speed ADC sampling.
  * Continuously acquires data from four microphones into shared buffers.
  * Stores the shared-buffer address in Flash for access by CM7-1.

* **CM7-1**

  * Runs the main control loop.
  * Reads sensor data and calculates the target direction.
  * Generates PWM outputs for the steering servo and drive motor.

Inter-core data sharing is implemented using `SCB_CleanInvalidateDCache()` for D-Cache synchronization together with Flash-stored buffer pointer addresses.

---

# Outdoor Mode — GPS Waypoint Navigation

## Operating Workflow

### 1. Waypoint Recording

Press **KEY3** to record the current GPS coordinates. Each waypoint is calculated as the average of **50 GPS samples**.

Press **KEY4** to finish waypoint recording.

### 2. Autonomous Navigation

The vehicle sequentially navigates to each recorded waypoint.

A waypoint is considered reached when either:

* Distance to the waypoint is **≤ 2 m**, or
* The heading deviation exceeds **90°**.

The controller then automatically switches to the next waypoint. After all waypoints have been completed, the vehicle stops.

---

## Core Algorithms

### Steering Servo PD Controller — `PID.c`

```text
Target heading = get_two_points_azimuth(current GPS position, next waypoint)
Feedback       = gnss.direction (GPS heading)
Error          = Target heading - Feedback
                 (with 360°/0° wrap-around handling)

Output = mid - (Kp × error + Kd × Δerror)

Output range: [520, 1020] PWM counts
Servo frequency: 50 Hz
```

### IMU Attitude Estimation — `Gyroscope_solve.c` *(Optional)*

* **Mahony complementary filter**

  * Quaternion integration
  * PI correction based on accelerometer cross-product error

* **1D Kalman filter**

  * Fuses accelerometer and gyroscope measurements
  * Estimates the roll angle

---

## Peripherals

| Peripheral               | Function                                         |
| ------------------------ | ------------------------------------------------ |
| **UART_2 (P10_0/P10_1)** | GNSS module — TAU1201 GPS receiver               |
| **UART_1 (P04_0/P04_1)** | Wireless module / external digital receiver      |
| **UART_4**               | Remote-control receiver — SBUS / custom protocol |
| **TCPWM_CH11 (P01_1)**   | Steering servo — 50 Hz                           |
| **TCPWM_CH25 (P09_1)**   | Drive motor — 1000 Hz                            |
| **GPIO P20_0–P20_3**     | KEY1–KEY4 buttons                                |
| **IPS114 SPI display**   | Debug information display                        |
| **PIT_CH0**              | 10 ms timer → `ServoPID()` interrupt             |
| **Flash**                | Persistent storage for waypoint coordinates      |

---

# Indoor Mode — Acoustic Source Localization and Navigation

## Operating Workflow

1. **CM7-0** continuously samples four microphone channels, with a **2048-sample buffer per channel**.
2. **CM7-1** performs GCC-SCOT cross-correlation on microphone signal pairs to calculate the **Time Difference of Arrival (TDOA)**.
3. Four TDOA measurements are converted into angular estimates. Their average is used as the acoustic source direction, `dir_num`, in the range **±180°**.
4. The steering servo is controlled according to the estimated direction.
5. When `|dir_num| > 100°`, the reverse-driving logic is triggered.

---

## Core Algorithm

### GCC-SCOT Acoustic Source Localization — `voice.c`

```text
For each microphone pair:

1. Perform a 2048-point FFT on both signals
   using CMSIS-DSP `arm_cfft_f32`.

2. Calculate the cross-power spectrum
   and apply SCOT weighting and normalization.

3. Perform an IFFT to obtain the
   generalized cross-correlation function.

4. Locate the correlation peak to determine
   the TDOA (sample delay).

5. Convert the time delay into an angle
   using `atan2`.

Final direction:
    Average the four angular estimates.
    Outliers are removed using `norightangle()`.
```

### Steering Output

```text
out = 740 - kp × dir_num

When |dir_num| > 100°:
    GPIO P09_0 = HIGH
    → Activate the reverse-drive relay
```

---

## Peripherals

| Peripheral                                     | Function                                    |
| ---------------------------------------------- | ------------------------------------------- |
| **ADC1_CH25, ADC1_CH00, ADC1_CH09, ADC0_CH01** | Four-channel microphone sampling            |
| **TCPWM_CH11 (P01_1)**                         | Steering servo — 50 Hz                      |
| **TCPWM_CH25 (P09_1)**                         | Drive motor — 1000 Hz                       |
| **GPIO P09_0**                                 | Reverse-direction relay                     |
| **PIT_CH0**                                    | 10 ms timer → steering control interrupt    |
| **PIT_CH2**                                    | 100 ms timer → UART digital polling         |
| **Flash**                                      | Persistent storage for ADC buffer addresses |

---

# Fast Math Library — `mymath.c`

| Function          | Description                                                              |
| ----------------- | ------------------------------------------------------------------------ |
| `myRsqrt()`       | Quake III fast inverse square root using the magic constant `0x5f375a86` |
| `fast_atan2()`    | 256-entry lookup table with linear interpolation                         |
| `arcsin_lookup()` | Precomputed 1000-entry arcsine lookup table                              |
| `KalmanFilter()`  | Generic 1D Kalman filter                                                 |

---

# Build and Flash

1. Open the corresponding `.eww` workspace in **IAR Embedded Workbench for ARM 9.40.1**:

   * **Outdoor:** `outdoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`
   * **Indoor:** `indoor/Seekfree_CYT4BB_Opensource_Library/project/iar/`

2. Select the target configuration for each core:

   * CM0+
   * CM7-0
   * CM7-1

3. Build the three core projects in sequence.

4. Flash the firmware for all three cores using a **J-Link** or the board's onboard debugger.

5. After power-on, **CM0+ automatically boots and starts the two CM7 cores**.

---

# Key Source Files

| File                             | Description                                         |
| -------------------------------- | --------------------------------------------------- |
| `project/user/main_cm0plus.c`    | CM0+ boot core; initializes and boots the CM7 cores |
| `project/user/main_cm7_0.c`      | CM7-0; main ADC sampling loop                       |
| `project/user/main_cm7_1.c`      | CM7-1; main control loop                            |
| `project/user/cm7_1_isr.c`       | CM7-1 interrupt service routines                    |
| `project/code/GPST.c`            | GPS waypoint recording and tracking                 |
| `project/code/PID.c`             | Steering servo PD controller                        |
| `project/code/voice.c`           | GCC-SCOT acoustic source localization               |
| `project/code/Gyroscope_solve.c` | Mahony + Kalman IMU attitude estimation             |
| `project/code/mymath.c`          | Fast mathematical utility functions                 |
| `project/code/init.c`            | Hardware peripheral initialization                  |

---

# License

This project is released under the **[GPL v3.0](outdoor/LICENSE)** license.

The underlying hardware abstraction and driver library are based on the **Seekfree Technology CYT4BB open-source library**.
