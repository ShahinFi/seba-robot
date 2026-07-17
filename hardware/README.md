# SEBA-ROBOT Hardware

This document describes the current electrical hardware, power system, electronic connections, and physical construction of SEBA-ROBOT.

The robot uses a Raspberry Pi 5 for high-level software and an STM32G474RE for real-time control. The STM32 reads the wheel encoders, BNO085 IMU, and two external motor-current sensors, and controls a Pololu Dual G2 motor-driver shield. The shield drives two geared DC motors.

The electronics are arranged across three physical layers. The motors are mounted at the bottom of the robot and are not counted as an electronics layer.

---

## 1. Hardware Overview

The Raspberry Pi 5 is the robot’s high-level computer, running ROS 2 and other application software, coordinating autonomous behavior, processing navigation and sensor data, communicating with external services, and exchanging motion commands, measurements, status, and fault information with the STM32.

The STM32 performs the real-time control. It controls both channels of the motor-driver shield and directly reads the IMU, motor encoders, motor-current measurements, and fault signals.

Each motor-driver output passes through the high-current path of an ACS711 current sensor before reaching one motor lead. The encoder integrated into each motor assembly connects directly to the STM32.

```text
+----------------------+
|   Raspberry Pi 5     |
| High-level computer  |
+----------+-----------+
           ^
           | UART communication
           v
+----------+-----------+
|   STM32G474RE        |<-------------------------------+
| Real-time control    |                                |
+----+-------------+---+                                |
     ^             ^                                    |
     | IMU data    | Motor-control signals              |
     v             v                                    |
+----+--------+  +----------------------+               |
| BNO085 IMU  |  | Pololu Dual G2       |               |
+-------------+  | motor-driver shield  |               |
                 +----------+-----------+               |
                            |                           |
                            | Motor output              |
                            v                           |
                   +--------+---------+                 |
                   |  ACS711 sensors  |-----------------+
                   +------------------+   VOUT, FAULT   |
                            |                           |
                            | Motor power path          |
                            v                           |
                     +------+------+                    |
                     |   motors    |--------------------+
                     | + encoders  |   Encoder A/B
                     +-------------+
```

The main interfaces are:

- Raspberry Pi and STM32: bidirectional UART communication
- BNO085 to STM32: IMU data and interrupt signal
- STM32 to BNO085: I2C clock and control communication
- STM32 to motor shield: PWM, direction, and sleep signals
- motor shield to STM32: motor-driver fault signals
- motor shield to motors: motor outputs through the ACS711 current paths
- ACS711 sensors to STM32: analog current measurements and fault signals
- wheel encoders to STM32: quadrature encoder channels

The complete power system, including the battery, protection, regulator, supply branches, voltage rails, and grounding, is described in the Power System section.

---

## 2. Main Components

### Real-time controller

- STMicroelectronics NUCLEO-G474RE
- STM32G474RE microcontroller

The STM32 performs:

- real-time balancing control
- motor PWM, direction, and sleep control
- hardware wheel-encoder counting
- BNO085 data acquisition
- ACS711 motor-current measurement
- motor-driver and current-sensor fault handling
- low-level forward-velocity and yaw-rate control
- communication with the Raspberry Pi

### High-level computer

- Raspberry Pi 5
- cooling fan installed

The Raspberry Pi is the robot’s high-level computer. It can run ROS 2 and other application software, coordinate autonomous behavior, process navigation and sensor data, communicate with external services, and perform other tasks that do not require hard real-time execution.

It exchanges motion commands, measurements, status, fault information, and other telemetry with the STM32.

### Motor driver

- Pololu Dual G2 High-Power Motor Driver 18v18 Shield

The shield contains two independent H-bridge channels. It receives PWM, direction, and sleep signals from the STM32, returns one fault signal from each channel, and drives the two motors from the battery-voltage motor supply.

### Motors

Two DFRobot FIT0186 motor assemblies are used.

Exact motor model:

```text
GB37Y3530-12V-251R
```

Relevant specifications:

- rated voltage: 12 V
- gear ratio: 43.8:1
- no-load speed: 251 rpm ±10%
- no-load current: approximately 350 mA
- stall current: approximately 7 A
- stall torque: approximately 18 kg·cm
- Hall-effect quadrature encoder
- encoder supply: 5 V
- encoder resolution: 16 CPR at the motor shaft and 700 CPR at the gearbox output

### IMU

- Adafruit BNO085 breakout

The BNO085 connects directly to the STM32 through I2C and an interrupt signal.

### Motor-current sensors

- 2 × Pololu ACS711EX ±15.5 A current-sensor carriers

One sensor measures the left motor current and one measures the right motor current.

### Electronics regulator

- Pololu D24V90F5
- fixed 5 V output
- available output current: approximately 9 A, depending on input voltage and thermal conditions

### Battery and main power control

- 3S LiPo battery
- nominal voltage: 11.1 V
- capacity: 5000 mAh
- discharge rating: 50C
- XT60 battery connection
- 20 A fuse
- main power button

---

## 3. Power System

The 3S LiPo battery supplies two power branches:

- direct battery power for the Pololu Dual G2 motor-driver shield
- regulated 5 V power for the Nucleo and Raspberry Pi 5

The positive battery conductor passes through the 20 A fuse and main power button before reaching the main distribution point. The battery GND conductor bypasses the fuse and button and connects directly to the same distribution point.

```text
                                          3S LiPo battery
                   +-----------------------------------------------------------+
                   |                                                           |
             Positive: 12 AWG                                                GND: 12 AWG
                   |                                                           |
                   v                                                           v
                   +-----------------------------------------------------------+
                   |                       XT60 connector                      |
                   +-----------------------------------------------------------+
                   |                                                           |
             Positive: 14 AWG                                                GND: 14 AWG
                   |                                                           |
                   |                                                           |
                   |                                                           |
                   v                                                           |
             +------------+                                                    |
             |  20 A fuse |                                                    |
             +------------+                                                    |
                   |                                                           |
                 14 AWG                                                        |
                   |                                                           |
                   v                                                           |
      +---------------------------+                                            |
      |     Main power button     |                                            |
      +---------------------------+                                            |
                   |                                                           |
                 14 AWG                                                        |
                   |                                                           |
                   +--------------+                             +--------------+
                                  |                             |
                                  v                             v
                        +------------------------------------------------------+
                        |              Main distribution point                 |
                        +------------------------------------------------------+
                               |                                            |
           +-------------------+--------------+                             +------------------------+
           |                                  |                             |                        |
    Positive: 14 AWG                        GND: 14 AWG             Positive: 14 AWG               GND: 14 AWG
           |                                  |                             |                        |
           v                                  v                             v                        v
    +---------------------------------------------------------+         +--------------------------------+
    |                     Pololu Dual G2                      |         | Electronics distribution point |
    |                  motor-driver shield                    |         +--------------------------------+
    +---------------------------------------------------------+             |                        |
          |             |            |             |                 Positive: 18 AWG              GND: 18 AWG
        M1A: 18 AWG  M1B: 18 AWG   M2A: 18 AWG   M2B: 18 AWG                |                        |
          v             |            |             v                        v                        v
+--------------------+  |            |  +---------------------+          +------------------------------+
| Left ACS711 sensor |  |            |  | Right ACS711 sensor |          |   Pololu D24V90F5 regulator  |
| high-current path  |  |            |  | high-current path   |          |           VIN / GND          |
+--------------------+  |            |  +---------------------+          +------------------------------+
          |             |            |             |                        |                        |
        18 AWG        18 AWG       18 AWG        18 AWG               +5 V: 18 AWG              GND: 18 AWG
          |             |            |             |                        |                        |
          v             v            v             v                        +------------+-----------+
+----------------------------+    +---------------------------+                          |
|         Left motor         |    |        Right motor        |                          v
+----------------------------+    +---------------------------+            +---------------------------+
                                                                           |   5 V distribution point  |
                                                                           +---------------------------+
                                                                                  |            |
                                                                        +---------+            +----------+
                                                                        |                                 |
                                                            +5 V: 18 AWG / GND: 18 AWG           5 A USB-C power cable
                                                                        |                                 |
                                                                        v                                 v
                                                         +---------------------------+      +---------------------------+
                                                         |  Nucleo CN7-6 / E5V       |      |      Raspberry Pi 5       |
                                                         |  Nucleo CN7-8 / GND       |      |        USB-C power        |
                                                         +---------------------------+      +---------------------------+
```

### 3.1 Battery protection and distribution

The battery leads between the battery and XT60 connector are 12 AWG.

After the XT60 connector, the positive and GND conductors are 14 AWG. The positive conductor passes through the 20 A fuse and main power button. The GND conductor runs directly to the main distribution point.

The main distribution point supplies two branches:

- a 14 AWG positive and GND pair for the Pololu Dual G2
- a 14 AWG positive and GND pair for the electronics distribution point

One active 18 AWG positive and GND pair from the electronics distribution point supplies the D24V90F5 regulator.

### 3.2 Motor power

The Pololu Dual G2 receives battery power directly from the main distribution point through a 14 AWG positive and GND pair.

The motor-output wiring is:

```text
M1A
-> left ACS711 high-current path
-> left motor

M1B
-> left motor

M2A
-> right motor

M2B
-> right ACS711 high-current path
-> right motor
```

All four motor-output conductors are 18 AWG.

`M1A` and `M1B` drive the left motor. `M2A` and `M2B` drive the right motor.

The H-bridge reverses the voltage polarity across each motor to reverse its direction. The motor-output conductors therefore do not have permanent positive or GND roles.

The motors are not powered from the 5 V regulator.

### 3.3 Regulated 5 V distribution

The D24V90F5 receives battery power through an 18 AWG positive and GND pair from the electronics distribution point.

The measured voltage at the regulator output is approximately 4.97 V.

The regulator output is divided into two active branches:

- an 18 AWG +5 V and GND pair to the Nucleo
- a 5 A-rated USB-C power cable to the Raspberry Pi 5

The Raspberry Pi is powered directly from the regulator branch and not through the Nucleo.

The USB-C cable rating describes the cable capacity. The current available to the Raspberry Pi also depends on the power-source configuration recognized by the Raspberry Pi.

### 3.4 Nucleo power rails

The Nucleo receives regulated 5 V through:

```text
D24V90F5 +5 V
-> Nucleo CN7-6 / E5V
```

The corresponding ground connection is:

```text
D24V90F5 GND
-> Nucleo CN7-8 / GND
```

Battery voltage must never be connected to `E5V`. The `E5V` input is supplied only from the regulated 5 V output.

The Nucleo board 5 V rail supplies both motor encoders:

```text
Nucleo CN7-18 / 5V
-> left encoder 5 V
-> right encoder 5 V
```

`CN7-18 / 5V` is a board power rail and not an STM32 GPIO signal.

The Nucleo 3.3 V rail supplies the BNO085 and both ACS711 current-sensor carriers:

```text
Nucleo CN7-16 / 3V3
-> BNO085 VIN
-> left ACS711 VCC
-> right ACS711 VCC
```

The BNO085 `3Vo` pin is not connected.

With a 3.3 V supply, the ACS711 analog outputs remain compatible with the STM32 ADC input range.

### 3.5 Common ground

The following are connected to the common robot ground:

- battery negative
- Pololu Dual G2 power GND
- D24V90F5 GND
- Nucleo GND
- Raspberry Pi GND through the USB-C power cable
- Raspberry Pi UART GND connection
- left and right encoder grounds
- BNO085 GND
- left and right ACS711 logic grounds

The common ground provides the voltage reference for UART, I2C, encoder, ADC, fault, PWM, direction, sleep, and interrupt signals.

`M1A`, `M1B`, `M2A`, and `M2B` are reversible H-bridge outputs and are not common-ground connections.

---

## 4. Electronic Connections

### 4.1 Raspberry Pi 5 and STM32G474RE

The Raspberry Pi and STM32 communicate through UART using two data signals and a common ground.

| Signal | Raspberry Pi connection | Nucleo connection | Direction |
|---|---|---|---|
| UART data | Physical pin 8 / GPIO14 / TX | CN7-2 / PC11 | Raspberry Pi to STM32 |
| UART data | Physical pin 10 / GPIO15 / RX | CN7-1 / PC10 | STM32 to Raspberry Pi |
| GND | Physical pin 6 / GND | Robot common GND | — |

The signal paths are:

```text
Raspberry Pi TX
-> STM32 PC11 / USART3_RX

STM32 PC10 / USART3_TX
-> Raspberry Pi RX

Raspberry Pi GND
-> robot common GND
```

PC11 and PC10 were selected because they provide the STM32 `USART3_RX` and `USART3_TX` functions.

Both boards use 3.3 V UART signals. No level shifter is required.

The UART carries commands from the Raspberry Pi and measurements, status, and fault information from the STM32.

The UART baud rate and packet format are defined in firmware and are not specified in this hardware document.

---

### 4.2 STM32G474RE and Pololu Dual G2 motor-driver shield

The motor-driver shield connects directly to the Nucleo through the Arduino-compatible shield headers.

The STM32 provides PWM, direction, and sleep control signals for both motor-driver channels. The shield provides one fault signal for each channel.

#### Left motor-driver channel

| Shield signal | Direction | STM32 pin | Nucleo position | Hardware role |
|---|---|---|---|---|
| `M1PWM` | STM32 to shield | PC7 | D9 | TIM3_CH2 PWM control |
| `M1DIR` | STM32 to shield | PA8 | D7 | Direction control |
| `M1SLP` | STM32 to shield | PA10 | D2 | Active-low sleep control |
| `M1FLT` | Shield to STM32 | PB10 | D6 | Active-low open-drain fault signal |

#### Right motor-driver channel

| Shield signal | Direction | STM32 pin | Nucleo position | Hardware role |
|---|---|---|---|---|
| `M2PWM` | STM32 to shield | PB6 | D10 | TIM4_CH1 PWM control |
| `M2DIR` | STM32 to shield | PA9 | D8 | Direction control |
| `M2SLP` | STM32 to shield | PB5 | D4 | Active-low sleep control |
| `M2FLT` | Shield to STM32 | PA6 | D12 | Active-low open-drain fault signal |

PC7 and PB6 were selected for the PWM signals because they provide the `TIM3_CH2` and `TIM4_CH1` timer-output functions.

The direction and sleep signals use STM32 GPIO outputs. The fault signals use STM32 GPIO inputs.

Driving `M1SLP` or `M2SLP` low disables the corresponding motor-driver channel.

The `M1FLT` and `M2FLT` outputs pull low while a motor-driver fault is present. These open-drain signals require pull-ups to 3.3 V.

The shield’s built-in `M1CS` and `M2CS` current-sense outputs are not used. Motor-current measurements are provided by the two external ACS711 current-sensor carriers.

#### PB6 startup requirement

PB6 also supports the STM32 `UCPD1_CC1` function. Its internal dead-battery pull-down can interfere with the `M2PWM` signal.

Firmware must disable this pull-down early during startup before PB6 is used for motor PWM. The required control bit is `UCPD1_DBDIS` in `PWR_CR3`.

---

### 4.3 Wheel encoders and STM32G474RE

Each motor assembly contains a Hall-effect quadrature encoder.

The encoder channels connect directly to STM32 timer inputs and do not pass through the motor-driver shield.

The motor cable uses the following wire order:

```text
Red | Black | Green | Blue | Yellow | White
```

| Wire | Function |
|---|---|
| Red | Motor lead |
| Black | Motor lead |
| Green | Encoder GND |
| Blue | Encoder 5 V |
| Yellow | Encoder channel A |
| White | Encoder channel B |

#### Left encoder

| Encoder signal | Nucleo connection | STM32 function |
|---|---|---|
| 5 V | CN7-18 / 5V | Encoder supply |
| GND | Robot common GND | Power and signal reference |
| Channel A | CN7-17 / PA15 | TIM2_CH1 |
| Channel B | CN10-31 / PB3 | TIM2_CH2 |

PA15 and PB3 were selected because they provide the `TIM2_CH1` and `TIM2_CH2` inputs required for the left quadrature encoder.

PA15 and PB3 share optional JTAG and SWO debug functions. In this robot they are assigned to the left encoder. PB3 therefore cannot also be used for SWO trace.

#### Right encoder

| Encoder signal | Nucleo connection | STM32 function |
|---|---|---|
| 5 V | CN7-18 / 5V | Encoder supply |
| GND | Robot common GND | Power and signal reference |
| Channel A | CN10-4 / PC6 | TIM8_CH1 |
| Channel B | CN10-3 / PB8 | TIM8_CH2 |

PC6 and PB8 were selected because they provide the `TIM8_CH1` and `TIM8_CH2` inputs required for the right quadrature encoder.

Channel A and channel B are identified by the installed wire colors. The resulting count sign also depends on the channel order and the mechanical orientation of each motor.

---

### 4.4 ACS711 current sensors and STM32G474RE

Each ACS711 current-sensor carrier has two electrically separate parts:

- a high-current path connected in series with one motor-output conductor
- low-voltage supply, analog-output, and fault connections

#### Left motor-current path

```text
M1A
-> left ACS711 high-current path
-> left motor

M1B
-> left motor directly
```

#### Left ACS711 connections

| Sensor signal | Nucleo connection | Hardware role |
|---|---|---|
| `VCC` | CN7-16 / 3V3 | Sensor supply |
| `GND` | Robot common GND | Power and signal reference |
| `VOUT` | CN7-32 / PA4 | Analog current signal |
| `FAULT` | CN7-35 / PC2 | Active-low latched fault signal |

PA4 was selected for `VOUT` because it provides the `ADC2_IN17` analog-input function.

PC2 was selected for `FAULT` because it can operate as a GPIO input and is associated with the `EXTI2` external-interrupt line.

#### Right motor-current path

```text
M2A
-> right motor directly

M2B
-> right ACS711 high-current path
-> right motor
```

#### Right ACS711 connections

| Sensor signal | Nucleo connection | Hardware role |
|---|---|---|
| `VCC` | CN7-16 / 3V3 | Sensor supply |
| `GND` | Robot common GND | Power and signal reference |
| `VOUT` | CN7-34 / PB0 | Analog current signal |
| `FAULT` | CN7-37 / PC3 | Active-low latched fault signal |

PB0 was selected for `VOUT` because it provides the `ADC1_IN15` and `ADC3_IN12` analog-input functions. The physical connection to PB0 is fixed; the ADC instance used to sample it is selected in firmware.

PC3 was selected for `FAULT` because it can operate as a GPIO input and is associated with the `EXTI3` external-interrupt line.

The separate `EXTI2` and `EXTI3` lines allow the two sensor fault signals to be detected independently.

With `VCC` at 3.3 V, the nominal zero-current voltage on `VOUT` is approximately 1.65 V.

Reversing the orientation of the ACS711 high-current terminals reverses the direction of the `VOUT` response.

The `FAULT` output latches low after an overcurrent event and is cleared by cycling power to the sensor.

---

### 4.5 BNO085 and STM32G474RE

The BNO085 connects to the STM32 through I2C and an interrupt signal.

| BNO085 signal | Nucleo connection | Hardware role |
|---|---|---|
| `VIN` | CN7-16 / 3V3 | IMU power |
| `GND` | Robot common GND | Power and signal reference |
| `SDA` | CN10-1 / PC9 | I2C3_SDA |
| `SCL` | CN10-2 / PC8 | I2C3_SCL |
| `INT` | CN10-18 / PB11 | Active-low interrupt signal |
| `RST` | Not connected | — |
| `3Vo` | Not connected | Breakout-board 3.3 V output |

PC9 and PC8 were selected because they provide the STM32 `I2C3_SDA` and `I2C3_SCL` functions.

PB11 was selected for `INT` because it can operate as a GPIO input and is associated with the `EXTI11` external-interrupt line.

`SDA` is the bidirectional I2C data line, `SCL` is driven by the STM32, and `INT` is an active-low interrupt signal driven by the BNO085.

The Adafruit breakout includes pull-up resistors on `SDA` and `SCL`. With `VIN` connected to 3.3 V, the I2C signals are compatible with the STM32’s 3.3 V logic.

The BNO085 `3Vo` pin is an output from the breakout board and is not used as its power input.

---

### 4.6 STM32 programming and debugging

The Nucleo’s onboard ST-LINK programs and debugs the STM32 through the Serial Wire Debug interface and the hardware reset signal.

| Debug signal | STM32 signal | Nucleo position | Hardware role |
|---|---|---|---|
| `SWDIO` | PA13 / SWDIO | CN7-13 | Bidirectional SWD data |
| `SWCLK` | PA14 / SWCLK | CN7-15 | SWD clock |
| `NRST` | NRST | CN7-14 | STM32 hardware reset |

PA13 and PA14 provide the dedicated `SWDIO` and `SWCLK` functions used by the onboard ST-LINK. `NRST` allows the debugger and programmer to reset the STM32.

---

## 5. Complete STM32 Connection Reference

### 5.1 STM32 signal connections

| Subsystem | Signal | Direction relative to STM32 | STM32 pin | Nucleo position | Peripheral or function |
|---|---|---|---|---|---|
| Raspberry Pi UART | Pi TX / STM32 RX | Input | PC11 | CN7-2 | USART3_RX |
| Raspberry Pi UART | STM32 TX / Pi RX | Output | PC10 | CN7-1 | USART3_TX |
| Pololu Dual G2 | M1PWM | Output | PC7 | D9 | TIM3_CH2 |
| Pololu Dual G2 | M1DIR | Output | PA8 | D7 | GPIO output |
| Pololu Dual G2 | M1SLP | Output | PA10 | D2 | GPIO output |
| Pololu Dual G2 | M1FLT | Input | PB10 | D6 | GPIO input |
| Pololu Dual G2 | M2PWM | Output | PB6 | D10 | TIM4_CH1 |
| Pololu Dual G2 | M2DIR | Output | PA9 | D8 | GPIO output |
| Pololu Dual G2 | M2SLP | Output | PB5 | D4 | GPIO output |
| Pololu Dual G2 | M2FLT | Input | PA6 | D12 | GPIO input |
| Left encoder | Channel A | Input | PA15 | CN7-17 | TIM2_CH1 |
| Left encoder | Channel B | Input | PB3 | CN10-31 | TIM2_CH2 |
| Right encoder | Channel A | Input | PC6 | CN10-4 | TIM8_CH1 |
| Right encoder | Channel B | Input | PB8 | CN10-3 | TIM8_CH2 |
| Left ACS711 | VOUT | Input | PA4 | CN7-32 | ADC2_IN17 |
| Left ACS711 | FAULT | Input | PC2 | CN7-35 | GPIO input / EXTI2 |
| Right ACS711 | VOUT | Input | PB0 | CN7-34 | ADC1_IN15 / ADC3_IN12 |
| Right ACS711 | FAULT | Input | PC3 | CN7-37 | GPIO input / EXTI3 |
| BNO085 | SDA | Bidirectional | PC9 | CN10-1 | I2C3_SDA |
| BNO085 | SCL | Output | PC8 | CN10-2 | I2C3_SCL |
| BNO085 | INT | Input | PB11 | CN10-18 | GPIO input / EXTI11 |

### 5.2 Nucleo power connections

| Supply or connection | Nucleo position | Purpose |
|---|---|---|
| Regulated +5 V input | CN7-6 / E5V | Nucleo external 5 V supply |
| Robot common GND | CN7-8 / GND | Power return and signal reference |
| Nucleo +5 V rail | CN7-18 / 5V | Powers both motor encoders |
| Nucleo +3.3 V rail | CN7-16 / 3V3 | Powers BNO085 VIN and both ACS711 VCC connections |

### 5.3 Programming and debug connections

| Signal | STM32 signal | Nucleo position |
|---|---|---|
| SWDIO | PA13 / SWDIO | CN7-13 |
| SWCLK | PA14 / SWCLK | CN7-15 |
| NRST | NRST | CN7-14 |

The onboard ST-LINK uses `SWDIO`, `SWCLK`, and `NRST` to program and debug the STM32.

---

## 6. Physical Construction

The electronics are arranged across three physical layers.

The motors are mounted at the bottom of the robot and are not considered an electronics layer.

### 6.1 Layer 1 — Battery and main power

Layer 1 contains:

- 3S LiPo battery
- XT60 battery connection
- 20 A fuse
- main push-button switch
- Pololu D24V90F5 regulator
- power terminals
- main power-distribution wiring

The positive battery conductor passes through the 20 A fuse and main power switch before reaching the main distribution point. From there, one branch supplies the motor-driver shield and the other supplies the electronics distribution point, which feeds the D24V90F5 regulator.

The main power connections use terminals to provide secure and removable connections.

### 6.2 Layer 2 — STM32 and motor electronics

Layer 2 contains:

- custom perfboard
- NUCLEO-G474RE
- Pololu Dual G2 motor-driver shield
- left ACS711 current sensor
- right ACS711 current sensor
- power terminals
- signal connector headers

The Nucleo is mounted above the perfboard, with its downward-facing `CN7` and `CN10` Morpho-header pins inserted into female sockets soldered to the perfboard.

The motor-driver shield is mounted above the Nucleo through the Arduino-compatible shield headers.

The two ACS711 current sensors are mounted on the perfboard.

The perfboard carries the permanent Layer 2 power and signal wiring.

External low-voltage cables use female Dupont connectors that mate with male header pins soldered onto the perfboard.

These removable connections include:

- left encoder 5 V, ground, channel A, and channel B
- right encoder 5 V, ground, channel A, and channel B
- BNO085 VIN, ground, SDA, SCL, and INT
- Raspberry Pi UART TX, RX, and ground

### 6.3 Layer 3 — Raspberry Pi and IMU

Layer 3 contains:

- Raspberry Pi 5
- Raspberry Pi cooling fan
- BNO085 IMU
- adjustable IMU mounting structure

The Raspberry Pi is powered through a 5 A-rated USB-C cable connected to a dedicated branch of the D24V90F5 5 V output.

The BNO085 is positioned near the center of Layer 3 beside the Raspberry Pi.

The IMU is mounted on an adjustable structure made from three pieces of the same aluminum framing used for the robot body.

The mounting structure allows adjustment of:

- the IMU position in one direction in the Layer 3 plane
- the IMU position in the perpendicular direction in the Layer 3 plane
- the IMU angle in the Layer 3 plane

This allows the IMU’s planar position and orientation relative to the robot frame to be adjusted mechanically.

### 6.4 Motors

The two motor assemblies are mounted at the bottom of the robot.

Each motor assembly contains:

- one geared DC motor
- one Hall-effect quadrature encoder
- two motor-power wires
- encoder 5 V
- encoder ground
- encoder channel A
- encoder channel B

The motor-power and encoder cables run between the motors and Layer 2.

The motor-power conductors and encoder conductors are connected separately.

