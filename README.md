# CYT4BB7 Auto-Driving Car: GPS and Sound-Direction Firmware

This repository archives two firmware projects for a competition car based on the Infineon CYT4BB7. The **outdoor** project records GPS waypoints and steers toward them. The **indoor** project estimates the direction of a sound source from four ADC microphone channels and controls steering and drive outputs. They are separate builds sharing a vendor platform, not a single GPS/audio fusion program.

> **Archive status:** The original equipment and parts are no longer available. This README reflects the included source and design files. It does not claim newly measured positioning accuracy, navigation completion rate, acoustic angular error, or continuous-operation reliability.

## Contents

- [Project scope](#project-scope)
- [Technology stack](#technology-stack)
- [Repository structure](#repository-structure)
- [Multicore architecture](#multicore-architecture)
- [Hardware interfaces](#hardware-interfaces)
- [Outdoor GPS mode](#outdoor-gps-mode)
- [Indoor sound-direction mode](#indoor-sound-direction-mode)
- [Build and flash reference](#build-and-flash-reference)
- [Known issues and verification status](#known-issues-and-verification-status)
- [License and third-party code](#license-and-third-party-code)

## Project scope

| Variant | Application behavior present in the source | Boundary |
| --- | --- | --- |
| Outdoor | Record waypoints from GNSS readings, estimate bearing to a selected waypoint, update steering by PD control, drive a motor by PWM | Waypoints are in RAM; the current waypoint-advance and end-of-route logic require correction before reuse |
| Indoor | Sample four analog microphones, estimate pairwise delays using FFT-based cross-correlation, calculate direction estimates, control steering and a reverse relay | The implemented spectral weighting does not realize the SCOT normalization described in the earlier README |

Both directories bundle a large Seekfree CYT4BB open-source library. Application-specific code is concentrated in each variant's **project/code/** and **project/user/** directories. The presence of SDK, driver, and template files should not be interpreted as project-specific implementation.

## Technology stack

| Layer | Technology represented in the repository |
| --- | --- |
| Microcontroller | Infineon CYT4BB7; two Cortex-M7 application cores and one Cortex-M0+ boot core |
| Language and IDE | Embedded C; IAR Embedded Workbench for ARM 9.40.1 as identified in source headers; separate checked-in IAR workspaces |
| Platform libraries | Seekfree CYT4BB open-source library and underlying Infineon Peripheral Driver Library |
| Digital signal processing | ARM CMSIS-DSP complex FFT/IFFT and math routines in the indoor sound-processing path |
| Outdoor sensing | TAU1201 GNSS receiver through UART; optional IMU processing functions are present but not enabled in the active navigation path |
| Indoor sensing | Four ADC microphone channels with 2048-sample arrays |
| Actuation and display | TCPWM steering-servo PWM, drive-motor PWM, reverse relay GPIO, IPS114 display, and GPIO keys |
| Hardware design | EasyEDA schematic and PCB JSON exports |

These are source-level dependencies and interfaces. No current toolchain installation or hardware compatibility test was performed.

## Repository structure

~~~text
Sound-Beacon-GPS-Navigation/
├── README.md
├── README.en.md             Link to this maintained English guide
├── hardware/                Schematic and PCB JSON exports
├── video&image/             Archived project media
├── outdoor/
│   ├── LICENSE
│   └── Seekfree_CYT4BB_Opensource_Library/
│       ├── project/code/    GPS, PD control, IMU, math, initialization
│       ├── project/user/    Per-core entry points and interrupts
│       └── project/iar/     IAR workspace
└── indoor/
    └── Seekfree_CYT4BB_Opensource_Library/
        ├── project/code/    Microphone acquisition, direction estimation, initialization
        ├── project/user/    Per-core entry points and interrupts
        └── project/iar/     IAR workspace
~~~

The source files most relevant to the application are:

| File | Role |
| --- | --- |
| [Outdoor GPST.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/GPST.c) | Waypoint recording and bearing calculations |
| [Outdoor PID.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/PID.c) | Steering PD controller |
| [Outdoor main_cm7_1.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) | Route progression and motor output |
| [Outdoor Gyroscope_solve.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/Gyroscope_solve.c) | IMU filter routines, not enabled in the active path |
| [Indoor voice.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/code/voice.c) | ADC data handling and sound-direction calculation |
| [Indoor main_cm7_0.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_0.c) | Four-channel sampling loop and shared-buffer address publication |
| [Indoor main_cm7_1.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) | Direction-driven steering, motor, and relay logic |

## Multicore architecture

### Indoor build

~~~text
Cortex-M0+ -- starts --> Cortex-M7-0 -- samples four ADC channels
       \                                |
        \                               +-- publishes buffer addresses
         \                                        |
          +-------- starts --> Cortex-M7-1 <-----+
                                  |
                                  +-- FFT/correlation and direction estimate
                                  +-- servo, motor, and reverse-relay outputs
~~~

The CM7-0 application writes shared-array addresses to a Flash page, and CM7-1 reads them. The microphone samples themselves are held in RAM, not stored as recordings in Flash. The code uses cache-maintenance calls around shared data access. These design choices require careful checking if ported to different memory layouts or startup sequences.

### Outdoor build

CM0+ also starts both M7 cores, but the outdoor CM7-0 application main loop is essentially idle. GNSS reception, waypoint handling, steering control, and motor output are on CM7-1. The indoor sampling diagram must not be applied to the outdoor build.

## Hardware interfaces

The following interfaces are named in the application code; physical wiring must be checked against the included [hardware exports](hardware/) before building a replacement.

| Interface | Use |
| --- | --- |
| UART_2, P10_0/P10_1 | Outdoor TAU1201 GNSS receiver |
| ADC1_CH25, ADC1_CH00, ADC1_CH09, ADC0_CH01 | Indoor four-channel microphone sampling |
| TCPWM_CH11, P01_1 | Steering servo, initialized at 50 Hz |
| TCPWM_CH25, P09_1 | Drive motor, initialized at 1000 Hz |
| GPIO P09_0 | Reverse-direction relay output in the indoor path |
| GPIO P20_0–P20_3 | Key inputs used by the mode-specific setup and controls |
| IPS114 | Debug display |

The same key labels are not assigned identically in every source file. Consult the relevant build's definitions before using physical buttons.

## Outdoor GPS mode

### Waypoint collection

At startup, [getpoint()](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/GPST.c) initializes the GNSS module and enters a key-driven collection loop. KEY3 calculates a waypoint from 50 GNSS parser calls and stores the average latitude and longitude in RAM arrays. KEY4 exits collection. The index is selected with the other keys in this code; bounds and GNSS validity are not rigorously checked.

The earlier README claimed Flash-backed waypoint persistence. The application does not write waypoint coordinates to Flash; a restart loses them.

### Steering and route progression

A 10 ms PIT_CH0 interrupt calls [ServoPID()](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/PID.c). Its feedback is the GNSS direction, and its target is the bearing from the current coordinate to the selected waypoint. The controller wraps heading error near ±180°, computes a proportional-plus-derivative steering value, and limits the servo PWM.

~~~text
target  = bearing(current GNSS position, selected waypoint)
error   = wrapped(target − GNSS direction)
output  = center PWM − Kp × error − Kd × error change
~~~

The CM7-1 main loop updates waypoint selection and motor PWM. It currently advances to the next waypoint when distance is at most **2 m** **or the absolute target bearing exceeds 90°**. That second test is not a heading-deviation test and can skip a waypoint unexpectedly. At the end, the index is reset to 1 rather than entering an explicit completed state. The code should be fixed and retested before being represented as reliable autonomous route completion.

The repository also contains Mahony and Kalman-filter implementations in Gyroscope_solve.c and math helpers in mymath.c. Their presence does not mean IMU fusion is active in this build: the relevant initialization and use in the navigation path are commented out.

## Indoor sound-direction mode

### Acquisition and estimation

CM7-0 repeatedly calls the ADC acquisition routine for four channels and fills arrays of 2048 samples. On CM7-1, [sound_project_fun_5()](indoor/Seekfree_CYT4BB_Opensource_Library/project/code/voice.c) subtracts channel means, computes four pairwise delay estimates, and forms four angles with atan2.

The per-pair routine converts the channels to complex arrays, performs 2048-point CMSIS-DSP FFTs, forms a cross-spectrum, applies an inverse FFT, and selects the peak lag. If any two of the four resulting angles differ by more than 90°, the function returns without changing the current direction. It does **not** remove just the outlying angle and average the rest. Otherwise the code averages angle magnitudes and assigns a sign using one delay.

### Spectral-weighting limitation

The previous README identified the method as GCC-SCOT. In the current [weight calculation](indoor/Seekfree_CYT4BB_Opensource_Library/project/code/voice.c), the multiplier is:

~~~text
sqrt( |cross-spectrum|² / (power(channel A) × power(channel B)) )
~~~

For the complex cross-spectrum formed just above it, numerator and denominator are mathematically equal in each bin. The multiplier is therefore approximately one, rather than the SCOT normalizer. The code also lacks a zero-denominator guard at that point. The implementation is accurately described as **FFT-based cross-correlation with an attempted SCOT weighting** until the formula is corrected and measured.

### Motion control

[Indoor main_cm7_1.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) updates steering PWM in the **main loop** from the estimated direction. It changes the reverse-relay output when the direction magnitude reaches roughly 100°. PIT_CH2 is configured for 100 ms polling of a UART-supplied control number. Indoor PIT_CH0 steering is not active: its initialization is commented out and its handler contains no controller call.

The source also has key-driven selection of sound-related control points and speed adjustments. A new build must validate the relay polarity, steering sign, speed limits, and microphone geometry.

## Build and flash reference

The checked-in workspaces are:

- [Outdoor cyt4bb7.eww](outdoor/Seekfree_CYT4BB_Opensource_Library/project/iar/cyt4bb7.eww)
- [Indoor cyt4bb7.eww](indoor/Seekfree_CYT4BB_Opensource_Library/project/iar/cyt4bb7.eww)

Historical source comments reference IAR Embedded Workbench for ARM 9.40.1. The intended workflow is to open the selected workspace, inspect the CM0+, CM7-0, and CM7-1 targets, build their images, and flash them with a compatible debugger. CM0+ starts the two application cores after boot.

This workflow has **not** been rebuilt or flashed in the present environment. Before attempting it, review the board pin mapping, the bundled vendor library, linker/shared-memory settings, microphone wiring, motor power stage, and mode-specific startup logic.

## License and third-party code

The repository includes [GPL v3 license text](outdoor/LICENSE) and the Seekfree CYT4BB open-source library, which itself wraps Infineon platform components. Preserve third-party notices and review the bundled license files when redistributing the source.
