# CYT4BB7 Competition Car: GPS and Sound-Direction Firmware

Firmware for an Infineon CYT4BB7 competition car with two operating builds:

- **Outdoor GPS navigation** records waypoints and steers the car toward them.
- **Indoor sound-direction navigation** estimates the direction of a sound source with four microphones and steers toward it.

Both builds were developed and tested on the physical car. They share the Seekfree CYT4BB platform library and use separate IAR workspaces.

## Contents

- [Features](#features)
- [Technology stack](#technology-stack)
- [System architecture](#system-architecture)
- [Hardware interfaces](#hardware-interfaces)
- [Repository structure](#repository-structure)
- [Outdoor GPS navigation](#outdoor-gps-navigation)
- [Indoor sound-direction navigation](#indoor-sound-direction-navigation)
- [Build and flash](#build-and-flash)
- [Controls and tuning](#controls-and-tuning)
- [License](#license)

## Features

| Build | Capabilities |
| --- | --- |
| Outdoor | GNSS waypoint recording, bearing calculation, 10 ms steering PD loop, motor PWM, IPS114 status display |
| Indoor | Four-channel ADC sampling, 2048-point FFT cross-correlation, directional steering, reverse-relay control, UART-controlled point selection |

## Technology stack

| Layer | Technology |
| --- | --- |
| Microcontroller | Infineon CYT4BB7 with two Cortex-M7 cores and one Cortex-M0+ core |
| Language and IDE | Embedded C; IAR Embedded Workbench for ARM 9.40.1 |
| Platform | Seekfree CYT4BB open-source library and Infineon Peripheral Driver Library |
| Signal processing | ARM CMSIS-DSP complex FFT/IFFT and math functions |
| Outdoor sensing | TAU1201 GNSS receiver over UART |
| Indoor sensing | Four ADC microphone channels |
| Actuation | TCPWM steering-servo and drive-motor outputs; GPIO reverse relay |
| Display and input | IPS114 display, GPIO keys, UART control input |
| Hardware design | EasyEDA schematic and PCB JSON exports |

## System architecture

The Cortex-M0+ core boots both Cortex-M7 application cores.

### Outdoor build

~~~text
TAU1201 GNSS --> CM7-1 --> waypoint selector --> steering PD --> servo PWM
                   |                                  |
                   +----------------------------------> motor PWM

CM7-0: idle application loop
CM0+: boots both M7 cores
~~~

### Indoor build

~~~text
Four microphones --> CM7-0 ADC sampling --> shared sample arrays
                                               |
                                               v
                                          CM7-1 FFT + delay estimation
                                               |
                                               +--> steering PWM
                                               +--> motor PWM
                                               +--> reverse relay

CM0+: boots both M7 cores
~~~

CM7-0 publishes the shared-array addresses through a Flash page; CM7-1 reads those addresses during startup. The microphone samples are held in RAM. Cache-maintenance calls coordinate access to shared data.

## Hardware interfaces

| Interface | Assignment |
| --- | --- |
| UART_2, P10_0/P10_1 | TAU1201 GNSS receiver in the outdoor build |
| ADC1_CH25, ADC1_CH00, ADC1_CH09, ADC0_CH01 | Four microphone inputs in the indoor build |
| TCPWM_CH11, P01_1 | Steering servo at 50 Hz |
| TCPWM_CH25, P09_1 | Drive motor at 1000 Hz |
| GPIO P09_0 | Reverse-direction relay |
| GPIO P20_0–P20_3 | Control keys |
| IPS114 | Onboard status display |

The [hardware directory](hardware/) contains EasyEDA schematic and PCB exports.

## Repository structure

~~~text
Sound-Beacon-GPS-Navigation/
├── README.md
├── README.en.md
├── hardware/                EasyEDA exports
├── video&image/             Project media
├── outdoor/
│   ├── LICENSE
│   └── Seekfree_CYT4BB_Opensource_Library/
│       ├── project/code/    GNSS, steering, IMU utilities, math
│       ├── project/user/    Core entry points and interrupts
│       └── project/iar/     IAR workspace
└── indoor/
    └── Seekfree_CYT4BB_Opensource_Library/
        ├── project/code/    Microphone acquisition and direction calculation
        ├── project/user/    Core entry points and interrupts
        └── project/iar/     IAR workspace
~~~

The Seekfree driver library is bundled in each build. The vehicle-specific modules are under **project/code/** and **project/user/**.

## Outdoor GPS navigation

### Waypoint recording

At startup, [GPST.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/GPST.c) initializes the GNSS receiver and enters the waypoint collection screen. KEY1 and KEY2 select a point index, KEY3 records a waypoint from the average of 50 GNSS readings, and KEY4 finishes collection. Waypoints are kept in RAM for the current run.

### Steering and drive control

A 10 ms PIT_CH0 interrupt calls [ServoPID()](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/PID.c). The target is the bearing from the current coordinate to the selected waypoint; feedback comes from the GNSS direction. The controller wraps angular error across ±180° and calculates a proportional-plus-derivative steering output.

~~~text
target  = bearing(current GPS position, selected waypoint)
error   = wrapped(target − GNSS direction)
output  = steering center − Kp × error − Kd × error change
~~~

[CM7-1 main](outdoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) updates the selected waypoint and motor PWM. It advances when distance to the point is at most 2 m or its absolute bearing exceeds 90°. After the final indexed point, the selector returns to point 1.

[Gyroscope_solve.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/Gyroscope_solve.c) contains Mahony and Kalman-filter routines for IMU experiments. The outdoor steering loop uses GNSS direction for feedback. Fast math helpers are in [mymath.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/mymath.c).

## Indoor sound-direction navigation

### Microphone acquisition

CM7-0 samples four microphone channels into 2048-sample arrays. [main_cm7_0.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_0.c) publishes their addresses for CM7-1, which reads them before entering the control loop.

### Direction calculation

[voice.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/code/voice.c) removes each channel's mean, calculates four pairwise time delays, and converts the delay combinations into four angles:

1. Convert two microphone windows to complex arrays.
2. Run 2048-point CMSIS-DSP FFTs.
3. Calculate the cross-spectrum and apply its weighting factor.
4. Run an inverse FFT and locate the correlation peak.
5. Convert the peak position to a signed sample delay.
6. Calculate direction estimates with atan2.

When the angle estimates differ by more than 90°, the previous direction is retained. Otherwise their magnitudes are averaged and a sign is assigned from one delay.

### Steering and motor output

[CM7-1 main](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) computes steering PWM from the sound direction in its main loop. Around ±100°, it changes the reverse-relay output. PIT_CH2 polls the UART-supplied control number every 100 ms. GPIO keys adjust speed and select sound-related control points.

## Build and flash

Open the workspace for the desired build in IAR Embedded Workbench for ARM 9.40.1:

- [Outdoor workspace](outdoor/Seekfree_CYT4BB_Opensource_Library/project/iar/cyt4bb7.eww)
- [Indoor workspace](indoor/Seekfree_CYT4BB_Opensource_Library/project/iar/cyt4bb7.eww)

Build the CM0+, CM7-0, and CM7-1 targets and flash the images with a compatible debugger. At power-on, CM0+ starts both M7 cores. Each workspace contains its own application and copy of the Seekfree platform library.

## Controls and tuning

| Control | Outdoor build | Indoor build |
| --- | --- | --- |
| KEY1 / KEY2 | Select a waypoint index during collection; adjust speed in the drive loop | Adjust speed and select a sound-control point during setup |
| KEY3 / KEY4 | Record a waypoint / finish waypoint collection | Mark a sound-control point / finish setup |
| Steering gain | Set in [PID.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/PID.c) | Set in [main_cm7_1.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/user/main_cm7_1.c) |
| PWM setup | [Outdoor init.c](outdoor/Seekfree_CYT4BB_Opensource_Library/project/code/init.c) | [Indoor init.c](indoor/Seekfree_CYT4BB_Opensource_Library/project/code/init.c) |

Adjust servo center, motor speed, microphone geometry, and waypoint selection for the vehicle configuration. Both builds show status on the IPS114 display.

## License

The repository includes [GPL v3 license text](outdoor/LICENSE) and the Seekfree CYT4BB open-source library. Keep the third-party notices when redistributing the project.
