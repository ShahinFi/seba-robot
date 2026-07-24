# SEBA-ROBOT STM32 Firmware

This document describes the STM32G474RE firmware used for SEBA-ROBOT real-time sensing, control, motor driving, safety handling, and serial communication.

The firmware runs on the STMicroelectronics NUCLEO-G474RE board using PlatformIO and STM32Cube. It reads the BNO085 IMU, wheel encoders, and ACS711 motor-current sensors; controls the Pololu Dual G2 motor-driver shield; estimates the robot state; runs the motion and actuator controllers; and exchanges commands, telemetry, acknowledgements, and event logs over serial.

The STM32 firmware owns the real-time robot behavior. The Raspberry Pi software sends operator commands and displays telemetry, but it does not run the real-time balance loop or motor-safety logic.

---

## 1. Firmware Overview

The firmware is organized around four runtime responsibilities:

- low-level hardware I/O for motors, encoders, IMU, current sensors, and serial communication
- fixed-rate state estimation from IMU and encoder measurements
- motion control and actuator current control for balance and motion tracking
- safety handling through state validity checks, fall detection, actuator faults, command watchdogs, and motor shutdown

The firmware interfaces with three external systems:

- the physical robot hardware documented in [SEBA-ROBOT Hardware](../../hardware/README.md)
- the control model documented in [Dynamics and Velocity-Tracking Control Model](../../control/README.md)
- the Raspberry Pi web software documented in [SEBA-ROBOT Raspberry Pi Software](../../raspberry-pi/README.md)

---

## 2. Build, Upload, and Monitor

The STM32 firmware is a PlatformIO project.

Project directory:

```bash
cd firmware/stm32
```

PlatformIO environment:

```text
nucleo_g474re
```

Board and framework:

| Item | Value |
|---|---|
| board | `nucleo_g474re` |
| framework | `stm32cube` |
| upload protocol | `stlink` |
| debug tool | `stlink` |
| monitor speed | `115200` |
| monitor line ending | `LF` |

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Open the serial monitor:

```bash
pio device monitor
```

The firmware uses the SH-2 library from:

```text
https://github.com/ceva-dsp/sh2.git#37e6f23
```

---

## 3. Runtime Architecture

The startup sequence in `src/main.c` initializes the firmware in this order:

1. STM32 HAL and system clock
2. motor driver outputs in a safe state
3. motor bring-up test state
4. wheel encoders
5. current sensors
6. actuator controller
7. motion controller
8. serial interface
9. BNO085 IMU
10. state estimation
11. serial console

Serial initialization happens before IMU initialization so IMU startup errors can be reported.

The main loop repeatedly:

- services pending IMU data
- processes serial console and Raspberry Pi commands
- processes pending reset requests
- updates event output
- writes periodic telemetry

Telemetry is streamed every:

```text
500 ms
```

The system clock is configured from the 16 MHz HSI oscillator with no PLL.

If a fatal startup initialization error occurs after motor initialization, the firmware stops both motors, disables both motor-driver channels, and remains in the error handler.

---

## 4. Source Layout

```text
src/
|-- commands/
|-- communication/
|-- console/
|-- control/
|   |-- actuator/
|   |-- motion_control/
|   `-- state_estimation/
|-- current_sensor/
|-- encoder/
|-- imu/
|-- motor/
|-- serial/
|-- tests/
`-- main.c
```

- `src/main.c` contains startup order, main-loop scheduling, telemetry streaming, system clock setup, and fatal error handling.
- `src/motor/` controls motor-driver PWM, direction, sleep, and stop behavior.
- `src/encoder/` maintains signed accumulated wheel positions from quadrature encoder hardware counters.
- `src/imu/` initializes and services the BNO085 SH-2 interface and converts IMU samples into robot-facing pitch, yaw, and angular-rate values.
- `src/current_sensor/` measures ACS711 motor current using ADC, DMA, timer-triggered sampling, zero-current offsets, and fault inputs.
- `src/control/state_estimation/` builds the robot state used by the controller.
- `src/control/motion_control/` runs the RSLQR motion controller and produces wheel-side torque references.
- `src/control/actuator/` converts wheel-side torque references into current references and PWM commands.
- `src/control/control_parameters.*` contains the physical parameters, timing parameters, filters, safety thresholds, torque limits, and RSLQR gain defaults.
- `src/commands/` parses and executes human-readable command groups.
- `src/communication/` implements acknowledged commands, telemetry lines, and event lines.
- `src/console/` receives serial lines and routes them to the command dispatcher or command protocol.
- `src/serial/` provides the serial read/write interface.
- `src/tests/` contains manual bring-up helpers for motors, encoders, IMU, and current sensors.

---

## 5. Serial Command and Telemetry Interface

The firmware accepts two command formats on the same serial interface.

The serial interface uses:

| Item | Value |
|---|---:|
| STM32 peripheral | `USART3` |
| transmit pin | `PC10` |
| receive pin | `PC11` |
| baud rate | `115200` |
| receive buffer | `256 bytes` |
| transmit buffer | `2048 bytes` |
| maximum command line | `159 characters` |

Command lines end with carriage return, line feed, or both. Non-printable input bytes are ignored except for carriage return, line feed, and backspace.

Direct console commands are plain text:

```text
balance start
state read
actuator status
```

The Raspberry Pi uses the acknowledged command format:

```text
CMD <id> <command text>
```

The STM32 replies with:

```text
ACK <id> OK
ACK <id> ERROR
```

Malformed acknowledged commands receive:

```text
ACK 0 ERROR
```

Recent command IDs are remembered. If the same command ID is received again, the firmware returns the remembered result instead of executing the command again.

Periodic telemetry uses:

```text
TEL key=value key=value ...
```

Event logs use:

```text
EVT ...
```

`TEL` lines are periodic snapshots. `EVT` lines are state-transition logs for important events such as balance arming and motion faults.

---

## 6. Command Reference

All commands can be entered directly through the PlatformIO serial monitor. The same command text can also be sent from the Raspberry Pi inside the `CMD <id> ...` protocol.

### 6.1 Motor Commands

| Command | Purpose |
|---|---|
| `motor left <-100...100>` | Sets the left motor open-loop PWM command in percent. Positive command drives forward. |
| `motor right <-100...100>` | Sets the right motor open-loop PWM command in percent. Positive command drives forward. |
| `motor stop` | Stops both motors and disables both motor-driver channels. |

Motor commands are manual bring-up commands. They bypass balance control and actuator current control.

### 6.2 Encoder Commands

| Command | Purpose |
|---|---|
| `encoder read` | Prints the accumulated left and right encoder positions. |
| `encoder reset` | Resets both accumulated encoder positions to zero at the current hardware counter values. |

### 6.3 Current Sensor Commands

| Command | Purpose |
|---|---|
| `current read` | Prints the latest left and right current-sensor readings, zero offsets, converted current, and fault state. |
| `current scope <left|right> <samples>` | Captures timed raw current samples for one current-sensor channel. |

`current scope` is a diagnostic command for observing sampled current behavior while PWM is active. Production actuator control uses the filtered readings from the current-sensor module.

### 6.4 IMU Commands

| Command | Purpose |
|---|---|
| `imu read` | Prints the latest decoded IMU values and validity flags. |

### 6.5 State Commands

| Command | Purpose |
|---|---|
| `state read` | Prints the estimated robot state, validity flags, and update counters. |

### 6.6 Actuator Commands

| Command | Purpose |
|---|---|
| `actuator left <mA>` | Enables actuator control and sets the left motor current reference. |
| `actuator right <mA>` | Enables actuator control and sets the right motor current reference. |
| `actuator both <left_mA> <right_mA>` | Enables actuator control and sets both motor current references. |
| `actuator torque <left_mNm> <right_mNm>` | Enables actuator control and sets wheel-side torque references. |
| `actuator stop` | Stops actuator current control and disables motor output. |
| `actuator status` | Prints actuator enable state, fault state, current references, measured currents, errors, integrator values, and PWM commands. |
| `actuator config` | Prints actuator control period, battery voltage, current gains, limits, and torque constant. |
| `actuator kp <mV_per_A>` | Sets the proportional current-control gain. |
| `actuator ki <mV_per_A_s>` | Sets the integral current-control gain. |
| `actuator battery <mV>` | Sets the battery voltage used to convert controller voltage to PWM command. |
| `actuator max-current <mA>` | Sets the maximum absolute current reference. |
| `actuator max-pwm <percent>` | Sets the maximum absolute PWM command. |
| `actuator integral-limit <mV>` | Sets the actuator current-loop integral limit. |
| `actuator period <ms>` | Sets the actuator control period. |
| `actuator torque-constant <mNm_per_A>` | Sets the wheel-side torque constant used to convert torque reference to current reference. |

### 6.7 Balance Commands

| Command | Purpose |
|---|---|
| `balance start` | Stops manual motor-test output and enables motion control if the current robot state is valid and inside the allowed fall angle. |
| `balance stop` | Disables motion control and actuator output. |
| `balance status` | Prints motion-control state, fault state, command values, torque commands, torque rates, gain scale, and torque limit. |
| `balance max-torque <mNm>` | Sets the symmetric left and right wheel-side torque limit. |
| `balance gain-scale <percent>` | Sets the multiplier applied to the RSLQR gain matrix. |
| `balance command <v_mps> <yaw_rate_rads>` | Sets the commanded forward velocity and yaw rate. |
| `balance gain <left|right> <0...5> <value>` | Sets one RSLQR gain entry for the left or right torque-rate row. |

Calling `balance start` while balance control is already running does not restart the controller state.

### 6.8 Telemetry and System Commands

| Command | Purpose |
|---|---|
| `telemetry read` | Prints one immediate `TEL` telemetry snapshot. |
| `system reset` | Requests a software reset of the STM32. |
| `help` | Prints the available command list. |

---

## 7. Telemetry and Event Fields

The telemetry stream is a single-line key/value snapshot:

```text
TEL valid=... imu=... imu_stale=... enc=... ...
```

### 7.1 State Validity and Timing

| Field | Meaning |
|---|---|
| `valid` | Complete robot state is valid. |
| `imu` | IMU orientation and gyroscope data are valid. |
| `imu_stale` | IMU orientation or gyroscope data exceeded its timeout. |
| `enc` | Encoder data is valid. |
| `updates` | State-estimator update count. |
| `ori_age` | Age of the latest orientation sample in milliseconds. |
| `gyro_age` | Age of the latest gyroscope sample in milliseconds. |
| `ori_count` | Orientation update count. |
| `gyro_count` | Gyroscope update count. |

### 7.2 Robot State

| Field | Meaning |
|---|---|
| `v` | Estimated forward velocity in m/s. |
| `theta` | Estimated pitch angle in radians. |
| `theta_dot` | Estimated pitch rate in rad/s. |
| `psi_dot` | Estimated yaw rate in rad/s. |
| `v_dot` | Estimated forward acceleration in m/s^2. |
| `theta_ddot` | Estimated pitch acceleration in rad/s^2. |
| `psi_ddot` | Estimated yaw acceleration in rad/s^2. |

### 7.3 Motion Control

| Field | Meaning |
|---|---|
| `balance` | Motion-control enable state. |
| `fault` | Motion-control fault-active state. |
| `fault_code` | Numeric motion-control fault code. |
| `fault_name` | Text motion-control fault name. |
| `cmd_age` | Age of the latest motion command in milliseconds. |
| `cmd_count` | Motion-command update count. |
| `fall` | Fall-detection state. |
| `v_cmd` | Commanded forward velocity in m/s. |
| `yaw_cmd` | Commanded yaw rate in rad/s. |
| `left_T` | Left wheel-side torque command in mNm. |
| `right_T` | Right wheel-side torque command in mNm. |
| `left_dT` | Left wheel-side torque-rate command in mNm/s. |
| `right_dT` | Right wheel-side torque-rate command in mNm/s. |
| `max_T` | Symmetric wheel-side torque limit in mNm. |
| `gain` | Motion-controller gain scale. |

### 7.4 Actuator Control

| Field | Meaning |
|---|---|
| `act` | Actuator-control enable state. |
| `left_ref` | Left motor current reference in mA. |
| `right_ref` | Right motor current reference in mA. |
| `left_meas` | Left measured motor current in mA. |
| `right_meas` | Right measured motor current in mA. |
| `left_pwm` | Left signed PWM command in per-mille. |
| `right_pwm` | Right signed PWM command in per-mille. |

### 7.5 Event Lines

Balance arming events report whether a start request was attempted, accepted, or rejected:

```text
EVT balance arm result=...
```

Rejected balance-arm events use one of these reasons:

| Reason | Meaning |
|---|---|
| `imu_stale` | IMU data exceeded the allowed age. |
| `imu_not_ready` | IMU state is not valid. |
| `encoder_not_ready` | Encoder state is not valid. |
| `fall_angle` | Pitch angle is outside the allowed start range. |
| `state_not_ready` | Complete robot state is not valid. |

Motion-fault events report the fault reason and the captured robot state at the fault:

```text
EVT motion fault reason=...
```

Fault-clear events report when the motion fault state is cleared:

```text
EVT motion fault-cleared reason=none
```

---

## 8. State Estimation

The state estimator runs at:

```text
200 Hz
```

The estimated robot state contains:

| State | Meaning |
|---|---|
| `v` | forward velocity |
| `theta` | body pitch angle |
| `theta_dot` | body pitch rate |
| `psi_dot` | yaw rate |
| `v_dot` | forward acceleration |
| `theta_ddot` | pitch acceleration |
| `psi_ddot` | yaw acceleration |

The estimator uses:

- BNO085 game-rotation-vector orientation
- BNO085 gyroscope angular velocity
- left and right wheel encoder positions

The enabled BNO085 reports are:

| Report | Period | Rate |
|---|---:|---:|
| accelerometer | `5000 us` | `200 Hz` |
| calibrated gyroscope | `5000 us` | `200 Hz` |
| game rotation vector | `10000 us` | `100 Hz` |

The BNO085 mounting convention used by the firmware is:

| Sensor axis | Robot direction |
|---|---|
| X | sideways |
| Y | forward |
| Z | upward |

Robot pitch is rotation about the sensor X axis. Robot yaw is rotation about the sensor Z axis.

IMU validity uses two timeout thresholds:

| Signal | Timeout |
|---|---:|
| orientation | `50 ms` |
| gyroscope | `50 ms` |

The estimator filters velocity and derivative estimates using:

| Parameter | Value |
|---|---:|
| velocity filter cutoff | `20 Hz` |
| derivative filter cutoff | `15 Hz` |

---

## 9. Motion and Actuator Control

The control implementation follows the motion-control and actuator-control split described in [Dynamics and Velocity-Tracking Control Model](../../control/README.md).

### 9.1 Motion Control

The motion controller runs at:

```text
200 Hz
```

The controller uses the augmented RSLQR signal:

```text
[v error, yaw-rate error, v_dot, theta_dot, theta_ddot, psi_ddot]
```

It produces left and right wheel-side torque-rate commands. The torque-rate commands are integrated to produce left and right wheel-side torque commands.

Default motion-control values:

| Parameter | Value |
|---|---:|
| maximum wheel torque | `5000 mNm` |
| fall angle | `1.047197551 rad` |
| motion command timeout | `300 ms` |
| gain scale | `2.5` |

Default RSLQR gain matrix:

```text
left  = [-2, -1, -2.8, -10, -1.2, -0.16]
right = [-2,  1, -2.8, -10, -1.2,  0.16]
```

### 9.2 Actuator Control

The actuator controller realizes wheel-side torque commands by regulating motor current.

Torque references are converted to current references using:

```text
current_reference = wheel_torque_reference / wheel_torque_constant
```

The actuator controller runs from a 1 kHz timer tick. The default control period is:

```text
1 ms
```

Default actuator values:

| Parameter | Value |
|---|---:|
| battery voltage | `11100 mV` |
| proportional gain | `3000 mV/A` |
| integral gain | `0 mV/(A*s)` |
| integral limit | `3000 mV` |
| maximum current reference | `5000 mA` |
| maximum PWM command | `100%` |
| wheel torque constant | `1000 mNm/A` |

The actuator disables itself if current readings fail or if either current sensor reports a fault.

---

## 10. Current Sensing

The firmware reads two ACS711 current sensors:

| Motor side | ADC input |
|---|---|
| left | `PA4 / ADC2_IN17` |
| right | `PB0 / ADC1_IN15` |

At startup, the firmware measures the zero-current ADC offset for each current sensor while the motors are stopped and disabled.

Current sensing uses timer-triggered ADC sampling and DMA:

| Parameter | Value |
|---|---:|
| PWM frequency reference | `20 kHz` |
| samples per PWM period | `4` |
| timed sample rate | `80 kHz` |
| DMA buffer samples | `80` |
| zero-current samples | `256` |
| ADC resolution | `12 bit` |
| ADC reference | `3300 mV` |
| ACS711 sensitivity | `90 mV/A` |

The current conversion applies the measured zero offset and the configured left/right current signs so positive measured current corresponds to positive wheel-side torque.

Current sign constants:

| Side | Sign |
|---|---:|
| left | `-1` |
| right | `-1` |

---

## 11. Motor and Encoder Conventions

Positive motor command means forward wheel drive for both sides.

Motor sign constants:

| Side | Sign |
|---|---:|
| left | `-1` |
| right | `1` |

The motor PWM frequency is:

```text
20 kHz
```

Encoder sign constants define positive accumulated encoder motion for the robot-forward direction:

| Side | Sign |
|---|---:|
| left | `1` |
| right | `-1` |

The encoder counters are 16-bit hardware counters. The firmware transfers wrapping hardware counts into signed accumulated positions when `Encoder_Update()` runs.

---

## 12. Safety and Fault Behavior

The STM32 owns real-time safety and motor shutdown behavior.

Motion-control faults:

| Fault | Meaning |
|---|---|
| `state_invalid` | The estimated robot state is not valid. |
| `imu_stale` | IMU orientation or gyroscope data is stale. |
| `fall` | Pitch angle exceeded the configured fall angle. |
| `actuator` | The actuator controller rejected or could not realize the torque reference. |

When motion control enters a fault:

- motion command state is reset
- torque command state is reset
- actuator control is disabled
- motor output is stopped through the actuator and motor layers
- the fault state is captured for event logging

`balance stop` disables motion control and actuator output, clears the motion-control fault state, and resets the motion command and torque command states.

`balance start` arms motion control only when the current robot state is valid and the pitch angle is inside the allowed fall angle.

The motion-command watchdog is separate from motion faults. If no fresh `balance command` arrives within:

```text
300 ms
```

the firmware sets the forward-velocity command and yaw-rate command to zero. This does not create a motion-control fault.

---

## 13. Control Parameters

The firmware keeps the main physical and control defaults in:

```text
src/control/control_parameters.c
```

Current physical and timing values:

| Parameter | Value |
|---|---:|
| body mass | `2.90 kg` |
| wheel radius | `0.035 m` |
| wheel separation | `0.10 m` |
| body center-of-mass height | `0.125 m` |
| body pitch inertia | `0.01752 kg*m^2` |
| body yaw inertia | `0.00483 kg*m^2` |
| wheel pitch inertia | `3.0625e-5 kg*m^2` |
| wheel yaw inertia | `1.90625e-5 kg*m^2` |
| wheel damping | `0.0005 N*m*s/rad` |
| encoder counts per wheel revolution | `2800` |
| state estimator rate | `200 Hz` |
| motion controller rate | `200 Hz` |

These values are used by the state estimator and motion controller. The hardware wiring and physical construction are documented separately in the hardware README.

---

## 14. Manual Bring-Up Tests

The `src/tests/` directory contains manual hardware bring-up helpers. These helpers are used through normal serial commands and are not separate firmware applications.

| Test area | Commands |
|---|---|
| motors | `motor left`, `motor right`, `motor stop` |
| encoders | `encoder read`, `encoder reset` |
| current sensors | `current read`, `current scope` |
| IMU | `imu read` |

Bring-up tests are intended to check wiring, signs, sensor readings, and low-level hardware behavior before closed-loop balance operation.

---

## 15. Troubleshooting

### No serial output

Check that the serial monitor is using:

```text
115200 baud
LF line ending
```

Check that the correct USB serial device or ST-LINK virtual COM port is selected.

### IMU initialization fails

Check:

- BNO085 power and ground
- I2C SDA and SCL wiring
- BNO085 interrupt wiring
- I2C address readiness during startup

The firmware prints IMU initialization errors over serial after serial initialization is complete.

### Telemetry shows `imu_stale`

`imu_stale=1` means the latest orientation or gyroscope sample exceeded the configured timeout. Check the BNO085 connection, interrupt signal, and whether `IMU_Process()` is being serviced by the main loop.

### Balance starts and immediately faults

Check the `EVT motion fault reason=...` line and the captured state fields. Common causes are:

- invalid state
- stale IMU data
- pitch angle outside the configured fall angle
- actuator current-control fault

### Motors do not move under actuator command

Check:

- actuator is enabled
- current reference is nonzero
- measured current changes with command
- PWM command is nonzero
- motor-driver fault inputs are normal
- current reference is inside the configured current limit

### Current sign is wrong

Positive measured current must correspond to positive wheel-side torque. If the sign is wrong, check current-sensor high-current path orientation and the left/right current sign constants in `src/current_sensor/current_sensor.c`.

### Raspberry Pi webpage shows stale telemetry

Check:

- STM32 firmware is running
- STM32 serial output contains `TEL` lines
- Raspberry Pi UART wiring matches the hardware README
- both boards share robot common ground
- no other process is holding the Raspberry Pi serial device

---

## 16. Related Documentation

- [SEBA-ROBOT Hardware](../../hardware/README.md)
- [Dynamics and Velocity-Tracking Control Model](../../control/README.md)
- [SEBA-ROBOT Raspberry Pi Software](../../raspberry-pi/README.md)
