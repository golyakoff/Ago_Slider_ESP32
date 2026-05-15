# AGO Slider – Firmware for 3‑Axis Linear/Rotary Motion Controller

## Overview

AGO Slider is an ESP32‑based firmware designed to control three independent axes (one linear – X, two rotary – C and B). It is intended for camera sliders, pan‑tilt heads, or any multi‑axis motion system. The firmware exposes a comprehensive BLE GATT server that allows:

- Real‑time position control (relative moves)
- Homing to limit switches (simultaneous or per‑axis)
- Configuration of motor parameters (microsteps, run/hold current, StealthChop, direction inversion)
- Power monitoring (voltage, current, power) via INA219
- Limit switch status monitoring (end‑stop sensors)
- Over‑the‑air (OTA) firmware updates via BLE

All settings are stored in NVS flash and survive power cycles.

## Hardware Requirements

- **MCU:** ESP32 (any variant with sufficient Flash, tested on ESP32‑WROOM‑32D)
- **Stepper Drivers:** 3× TMC2130 (SPI interface)
- **Stepper Motors:** 3× NEMA17 (or compatible)
- **End‑stop sensors:** 3× (active LOW, e.g. magnetic or mechanical, wired to PCA9555 I/O expander)
- **I/O Expander:** PCA9555 (I²C, address 0x20)
- **Power Monitor:** INA219 (I²C, address 0x40, shunt 0.02 Ω recommended for up to 5 A)
- **Additional:** RGB LED (WS2812B, optional) for status indication

### Pin assignments (default)

| Function            | GPIO |
|---------------------|------|
| SPI SCLK            | 14   |
| SPI MOSI            | 13   |
| SPI MISO            | 12   |
| Global enable       | 15   |
| X axis CS           | 21   |
| X axis STEP         | 22   |
| X axis DIR          | 23   |
| C axis CS           | 17   |
| C axis STEP         | 19   |
| C axis DIR          | 18   |
| B axis CS           | 25   |
| B axis STEP         | 27   |
| B axis DIR          | 26   |
| RGB LED (WS2812B)   | 16   |
| I²C SDA             | 32   |
| I²C SCL             | 33   |
| PCA9555 interrupt   | 35   |

> **Note:** Pinout can be easily changed in `main/app_main.cpp`.

## BLE GATT Service

### Service UUID: `0xFE95`

All characteristics use 16‑bit UUIDs.

#### Control & Status

| UUID  | Characteristic   | Properties            | Description |
|-------|------------------|-----------------------|-------------|
| 0xF001 | MOT_EN           | Read / Write          | Enable all motors (1=on, 0=off) |
| 0xF002 | HOME             | Read / Write / Notify | Homing command & status (see byte format below) |
| 0xF003 | LIMIT            | Read / Notify         | Current limit switch states (bits 0‑2: X, C, B) |
| 0xF004 | MOVE             | Write                 | Relative move: 6 bytes (3× int16 little‑endian: X, C, B steps) |

#### Power Monitoring

| UUID  | Characteristic   | Properties            | Description |
|-------|------------------|-----------------------|-------------|
| 0xF020 | BATT_LEVEL       | Read / Notify         | Battery level (0‑255, maps 18 V → 0, 21.5 V → 255) |
| 0xF021 | PWR_INFO         | Read / Notify         | 12 bytes – 3× float (voltage, current, power) |
| 0xF022 | PWR_INFO_STR     | Read / Notify         | Human‑readable string, e.g. `21.48V 0.082A 1.76W` |

#### Configuration (Read/Write, no notify)

| UUID  | Characteristic   | Size | Description |
|-------|------------------|------|-------------|
| 0xF030 | MICROSTEPS       | 3    | Microstep setting per axis: 0=256,1=1,2=2,4,8,16,32,64 |
| 0xF031 | RUN_CURRENT      | 6    | Run current per axis (uint16 mA, little‑endian) |
| 0xF032 | HOLD_CURRENT     | 6    | Hold current per axis (uint16 mA) |
| 0xF033 | AXIS_UNIT        | 1    | Bits: 0=mm,1=deg for X (bit0), C (bit1), B (bit2) |
| 0xF034 | UNITS_PER_STEP   | 12   | 3 floats: mm/step or deg/step (little‑endian) |
| 0xF035 | AXIS_SPEED       | 6    | Default speed for moves (steps/sec, uint16 per axis) |
| 0xF036 | AXIS_ACCEL       | 6    | Acceleration (steps/sec², uint16 per axis) |
| 0xF037 | VIRTUAL_LIMIT    | 1    | Enable virtual limit (bits: X, C, B) |
| 0xF038 | STEALTHCHOP      | 1    | Enable StealthChop mode per axis (bits) |
| 0xF039 | INVERT_DIR       | 1    | Invert direction per axis (bits) |

#### OTA (Over‑the‑Air)

| UUID  | Characteristic   | Properties | Description |
|-------|------------------|------------|-------------|
| 0xF090 | VERSION          | Read / Notify | Firmware version string |
| 0xF091 | OTA_CONTROL      | Write      | Control commands (start, end, abort) |
| 0xF092 | OTA_DATA         | Write      | Firmware binary chunks |

**OTA control format**:
- `0x01` + 4 bytes (uint32_t little‑endian) → start update with given total size
- `0x02` → end update and reboot
- `0x03` → abort update

## Homing Protocol

- Write to `HOME` (0xF002) with **high nibble** bits (4‑6) to request homing for axes:  
  `0x10` = X, `0x20` = C, `0x40` = B, `0x70` = all.
- The device simultaneously moves all requested axes toward the limit switches (negative direction by default).
- When an axis limit switch is triggered, the motor stops immediately (`forceStop`).
- The `HOME` characteristic sends **notifications** after each axis finishes, updating both the high nibble (remaining axes) and low nibble (completed axes).
- Example sequence for homing all axes (order X → C → B):
  - Write `0x70` → notification `0x70` (all scheduled, none homed)
  - X finished → notification `0x61`
  - C finished → notification `0x43`
  - B finished → notification `0x07`

## Building & Flashing

### Prerequisites
- ESP‑IDF v5.5.4 (or compatible)
- Git, CMake, Ninja, Python

### Clone & Build
```bash
git clone https://github.com/your-repo/ago-slider-firmware.git
cd ago-slider-firmware
idf.py set-target esp32
idf.py menuconfig   # (optionally adjust partition table and BLE)
idf.py build
idf.py flash monitor

