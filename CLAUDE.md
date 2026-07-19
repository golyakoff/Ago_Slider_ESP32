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
`.vscode/settings.json`). To activate the environment in a shell, dot-source the EIM profile in
PowerShell: `. "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1"` (the plain
`export.ps1` fails — the Python venv lives under `C:\Espressif\tools`, not the default path).
See also the project skills: `flash` (build/flash/monitor) and `release` (tagging + CI).

```
idf.py set-target esp32
idf.py build
idf.py flash monitor
idf.py menuconfig     # adjust partition table / component config
```

There is no unit test suite in this repo — verification is build success plus on-device testing
(flash + monitor + exercise BLE characteristics). `sdkconfig.ci` exists but is currently empty.

Releases: pushing a `v*.*.*` tag triggers `.github/workflows/release.yml`, which builds debug and
release variants from `sdkconfig.defaults` + `sdkconfig.debug`/`sdkconfig.release` overlays
(CI ignores the committed `sdkconfig`, which serves local builds) and publishes
`ago_slider_{debug,release}_16mb_fw.bin` to a GitHub Release — the asset names the Android app's
update check expects. After menuconfig changes that CI should pick up, regenerate the defaults
with `idf.py save-defconfig`.

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
  the BLE stack directly.
- **app_config** — defines the packed `app_config_t` struct (the single source of truth for all
  persisted settings: microsteps, run/hold current, axis units, steps-per-unit, speed/accel,
  virtual limit, StealthChop, direction invert) plus pack/unpack helpers to/from raw byte arrays
  (matching the BLE characteristic wire format).
- **nvs_config** — persists `app_config_t` to NVS flash. `nvs_config_t` is a type alias of
  `app_config_t`. Each `nvs_config_set_*` setter mutates the in-memory config and saves to flash
  in one call. `app_main.cpp` loads config at boot and pushes it into both `ble` (so
  characteristic reads reflect persisted state) and `motion`/`tmc2130` (so hardware matches
  persisted state) before the rest of init runs.
- **ota** (`components/ota`) — BLE OTA write-to-flash logic on native `esp_ota_ops`
  (sequential-write mode, deferred reboot via `esp_timer`). Firmware version comes from the app
  descriptor via `git describe --tags` (PROJECT_VER is intentionally not set in CMake), exposed
  via `ota_get_current_version()`; a build from a release tag reports exactly that tag, which
  the Android app compares against. Rollback is enabled
  (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`) — `app_main()` calls `ota_confirm_running_image()`
  after successful init, otherwise the bootloader reverts to the previous image on the next
  reboot.

### Config flow

`app_config_t` (raw packed bytes) is the shared representation across three components:
BLE characteristic wire format <-> `app_config` pack/unpack helpers <-> `nvs_config` (flash) <->
`app_main.cpp` glue that also mirrors values into `tmc2130`/`motion`/`FastAccelStepper` runtime
state. When adding a new persisted setting, all four places need updating together: the struct in
`app_config.h`, a BLE characteristic + callback in `ble.h`/`ble.c`, an `nvs_config_set_*`
accessor, and the apply/init logic in `app_main.cpp`.

### Position, endstop events and hardware calibration

- POSITION (0xF005, read/notify) reports the step generators' commanded positions (3× int32 LE
  STEP pulses); homing anchors an axis via `forceStopAndNewPosition(0)`, so the switch is the
  firmware's zero. A publisher task in `app_main.cpp` notifies every 200 ms while anything moves.
- **PCA9555 single-reader invariant**: the interrupt task in `components/pca9555` is the ONLY
  code that reads the input registers; `pca9555_get_gpio_value()` serves inputs from its cached
  `pin_state`. Any direct register read elsewhere would clear the PCA9555 INT latch behind the
  task's back and desynchronise its change detection — that exact bug silently swallowed endstop
  events for minutes. The task also re-reads on a 500 ms timeout as a lost-edge safety net.
- The endstop sensors are magnetic and emit millisecond-long pulses on a passing carriage (a
  ~4 ms blink was measured at 10 mm/s), so anything that must stop on a sensor has to react
  on-device: `app_main`'s `on_limit_change` forwards every edge to
  `motion_on_limit_pin_event()`.
- CALIBRATE (0xF006, write/notify) runs the span measurement entirely in `motion`: fast seek to
  the min switch → retreat → slow re-seek (position zeroed at the trigger) → fast seek to the
  max switch → retreat → slow re-seek (span captured) → park at the commanded offset. Endstop
  edges drive the stops (~1 ms latency); retreat/park phases ignore them by design; the
  virtual-limit monitor is suppressed while calibrating; per-phase timeout 60 s. Write format:
  axis byte (0xFF = abort) + park offset + retreat (int32 LE each); notify: axis, phase
  (`motion_calib_phase_t`), span.

### Dependencies

Managed components (`managed_components/`, pinned in `main/idf_component.yml` /
`dependencies.lock`, gitignored): `espressif/led_strip`, `espressif/arduino-esp32` (used only for
`ledcAttach`/`ledcWrite` PWM fan control and `Arduino.h`/millis-style helpers pulled in by
FastAccelStepper), and `FastAccelStepper` (git dependency, pinned to `v0.33.13`).

Partition table is custom: `partitions/16MB.csv` (dual OTA app slots `app0`/`app1`, required for
BLE OTA to work).
