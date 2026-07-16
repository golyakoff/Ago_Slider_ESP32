# TODO

## Rework BLE OTA subsystem

The OTA implementation was ported from the `Matrix_Clock_ESP32` project (Arduino-based)
and needed to be redesigned for ESP-IDF.

- [x] Redesign the write path around native `esp_ota_ops` idioms: sequential-write mode
  (no long blocking erase in `ota_begin`), no per-chunk `vTaskDelay` hack, overflow check,
  reboot via one-shot `esp_timer` instead of `esp_restart()` inside the BLE callback.
- [x] Resolve the orphaned `components/ota` component — OTA module now lives there
  (moved out of `ble`), correct `PRIV_REQUIRES`, added to `main`'s REQUIRES.
- [x] Firmware version comes from the app descriptor via `git describe --tags` (no
  manual `PROJECT_VER` bump needed) — a build from a release tag gets exactly that tag,
  which is what the Android app compares against.
- [x] Rollback support: `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` was already on, but
  nothing ever confirmed the image — after an OTA the bootloader would roll back on the
  next reboot. `ota_confirm_running_image()` is now called at the end of `app_main()`.

- [x] GitHub Releases workflow (`.github/workflows/release.yml`): on a `v*.*.*` tag builds
  debug and release variants (`sdkconfig.defaults` + `sdkconfig.debug`/`sdkconfig.release`
  overlays) and publishes `ago_slider_debug_16mb_fw.bin` / `ago_slider_release_16mb_fw.bin`
  with the matching CHANGELOG.md section as release notes. NOTE: `sdkconfig.defaults` must
  be regenerated with `idf.py save-defconfig` after any menuconfig change that should
  affect CI builds.

Remaining:

- Reconsider the BLE transfer protocol (chunk size vs. MTU 517, missing ack/flow control,
  progress notifications back to the client) together with the Android app's
  `FirmwareRepository` — both sides must change in sync. The Android side also paces the
  transfer with a fixed 30 ms delay per 512-byte chunk — with ack/flow control that delay
  can go away.
