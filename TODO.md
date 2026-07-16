# TODO

## Rework BLE OTA subsystem

The OTA implementation (`components/ble/src/ota.c` + OTA characteristics in `ble.c`) was
ported from the `Matrix_Clock_ESP32` project, which was Arduino-based (`Arduino.h`/
`Update.h`), while this firmware is pure ESP-IDF — the internals need to be redesigned
for ESP-IDF instead of mimicking the Arduino flow. Points to cover:

- Redesign the write path around native `esp_ota_ops` idioms (it already calls
  `esp_ota_begin`/`esp_ota_write`, but the control flow / state machine is inherited from
  the Arduino version).
- Resolve the orphaned `components/ota` component: its CMakeLists references
  `src/ota.c` that doesn't exist and it isn't in `main`'s REQUIRES — either implement the
  OTA module there properly and move the logic out of the `ble` component, or delete it.
- Firmware version should come from the ESP-IDF app descriptor (`esp_app_desc_t` /
  `esp_app_get_description()`), set via project version in CMake, instead of a hand-kept
  string.
- Add post-update validation with rollback: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`,
  `esp_ota_mark_app_valid_cancel_rollback()` after a successful health check on first boot.
- Reconsider the BLE transfer protocol (chunk size vs. MTU 517, missing ack/flow control,
  progress notifications back to the client) together with the Android app's
  `FirmwareRepository` — both sides must change in sync.
- Set up a GitHub Releases workflow for this repo producing assets named
  `*_release_4mb_fw.bin` — that's what the Android app's update check expects.
