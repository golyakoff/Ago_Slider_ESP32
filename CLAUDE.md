# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

AGO Slider firmware: an ESP-IDF (v5.5.4+) application for an ESP32 that drives a 3-axis motion
controller (linear X, rotary C, rotary B) via TMC2130 stepper drivers, exposing control/config
over a custom BLE GATT service (including BLE OTA updates). See `README.md` for the full GATT
characteristic table (UUIDs, byte layouts) and the homing protocol — that reference is the
contract any BLE-facing change must stay compatible with.

## Build / flash commands

This is a standard ESP-IDF project (not Arduino/PlatformIO despite pulling in `arduino-esp32` as
a component). Requires the ESP-IDF v5.5.4 environment to be sourced (`idf.py` on PATH); on this
machine IDF lives at `C:\esp\v5.5.4\esp-idf` and the target COM port is `COM4` (see
`.vscode/settings.json`).

```
idf.py set-target esp32
idf.py build
idf.py flash monitor
idf.py menuconfig     # adjust partition table / component config
```

There is no unit test suite in this repo — verification is build success plus on-device testing
(flash + monitor + exercise BLE characteristics). `sdkconfig.ci` exists but is currently empty.

## Architecture

`main/app_main.cpp` is the composition root: it owns all GPIO/peripheral pin assignments (SPI
bus, per-axis CS/STEP/DIR, I2C, RGB LED), wires every component together via callbacks, and is
the only place that talks to more than one component at once. Read it first when tracing how a
BLE command reaches hardware. Pin assignments are documented in `README.md` and can be changed
only in `app_main.cpp`.

Everything else lives in `components/`, each an isolated ESP-IDF component (own
`CMakeLists.txt`, `include/`, `src/`) with dependencies declared via `REQUIRES` — check a
component's `CMakeLists.txt` to see what it's allowed to depend on before adding new includes.

- **tmc2130** — low-level SPI driver for the 3 TMC2130 stepper driver chips (register access,
  current/microstep/StealthChop/direction config). No knowledge of steps/motion planning.
- **motion** (C++) — motion controller sitting on top of `FastAccelStepper` (external managed
  component, does STEP/DIR pulse generation) and `pca9555` (limit switch reads). Owns
  homing state machine, relative moves, unit conversion (mm/deg <-> steps), virtual limits.
- **pca9555** — I2C I/O expander driver; limit switches are wired through it (active LOW) with
  interrupt-driven callbacks (`pca9555_set_interrupt_handler`).
- **ina219** — I2C power monitor driver (voltage/current/power), pushes readings via callback to
  `app_main.cpp`, which forwards them to BLE (battery %, power characteristics).
- **i2c_manager** — shared I2C bus/device handle setup used by `pca9555` and `ina219`.
- **ble** — the entire BLE GATT server (service `0xFE95`) plus BLE OTA control/data
  characteristics. `ble_init()` takes one callback per characteristic (see `ble.h`); this is the
  single entry/exit point between BLE and the rest of the firmware — `app_main.cpp` never touches
  the BLE/NimBLE stack directly. `ble/src/ota.c` implements the OTA write-to-flash logic invoked
  through those callbacks.
- **app_config** — defines the packed `app_config_t` struct (the single source of truth for all
  persisted settings: microsteps, run/hold current, axis units, steps-per-unit, speed/accel,
  virtual limit, StealthChop, direction invert) plus pack/unpack helpers to/from raw byte arrays
  (matching the BLE characteristic wire format).
- **nvs_config** — persists `app_config_t` to NVS flash. `nvs_config_t` is a type alias of
  `app_config_t`. Each `nvs_config_set_*` setter mutates the in-memory config and saves to flash
  in one call. `app_main.cpp` loads config at boot and pushes it into both `ble` (so
  characteristic reads reflect persisted state) and `motion`/`tmc2130` (so hardware matches
  persisted state) before the rest of init runs.
- **ota** (`components/ota`) — currently an empty/orphaned component (CMakeLists references
  `src/ota.c` but no source files exist and it is not in `main`'s `REQUIRES`). The real OTA
  implementation is `ble/src/ota.c`. Don't assume this component builds.

### Config flow

`app_config_t` (raw packed bytes) is the shared representation across three components:
BLE characteristic wire format <-> `app_config` pack/unpack helpers <-> `nvs_config` (flash) <->
`app_main.cpp` glue that also mirrors values into `tmc2130`/`motion`/`FastAccelStepper` runtime
state. When adding a new persisted setting, all four places need updating together: the struct in
`app_config.h`, a BLE characteristic + callback in `ble.h`/`ble.c`, an `nvs_config_set_*`
accessor, and the apply/init logic in `app_main.cpp`.

### Dependencies

Managed components (`managed_components/`, pinned in `main/idf_component.yml` /
`dependencies.lock`, gitignored): `espressif/led_strip`, `espressif/arduino-esp32` (used only for
`ledcAttach`/`ledcWrite` PWM fan control and `Arduino.h`/millis-style helpers pulled in by
FastAccelStepper), and `FastAccelStepper` (git dependency, pinned to `v0.33.13`).

Partition table is custom: `partitions/16MB.csv` (dual OTA app slots `app0`/`app1`, required for
BLE OTA to work).
