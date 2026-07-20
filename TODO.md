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

- [x] OTA transfer flow control (firmware v0.1.2 + app v0.1.1): OTA_DATA/OTA_CONTROL are
  now `ESP_GATT_RSP_BY_APP` — the ATT write response is sent only after the chunk hits
  flash (for the "end" command — after image validation), so the phone's awaited write
  response IS the ack. The app sends chunks of MTU-3 bytes (single Write Request instead
  of the previous 512-byte Prepare/Execute long writes — 3 round trips each) and the
  fixed 30 ms per-chunk delay is gone. Prepared long writes are still supported firmware-
  side (buffer + EXEC_WRITE) for compatibility with app v0.1.0.

## Focus scenario: C lags at the end of a pass

Measured on hardware 2026-07-20 (60 s pass, 770 mm of X travel, subject 27 cm away): C
tracks the subject to within 4 pulses (0.05 deg) for about 95% of the run, then falls
behind by 49 pulses (0.65 deg) over the last 1.5 s. In frame that is a slight slide off
centre in the closing second — invisible on a normal lens, but not on a long one.

The error appears exactly when X begins decelerating. The feed-forward term is proportional
to X's instantaneous speed, so it vanishes as X stops, leaving only the correction term,
whose 3 s time constant is far too slow for a ~1.5 s deceleration ramp.

That explanation is incomplete, and the gap matters: the captured log shows C's position
frozen at 2767 pulses for the final 1.5 s while the commanded speed was still non-zero
(66 -> 43 -> 16 pulses/s). Something stopped the axis outright rather than merely slowing
it, and the feed-forward argument does not account for that. **Start here**, not with
tuning.

Rejected: a settle phase after X stops. It would force the user to pad every shot with
footage they do not want.

Candidate directions:
- Explain the freeze first — instrument `motion_axis_set_speed` / the ramp generator's
  state around the deceleration, and check whether `applySpeedAcceleration()` behaves as
  assumed when the speed cap falls steeply while running continuously.
- Shorten the ramp: the error accumulates over the deceleration, so less time in it means
  less to accumulate. The scenario currently computes its own ramp (5% of duration, capped
  at 2 s) and deliberately ignores the stored settings, so this needs a new parameter
  before it is a lever the user can reach.
- Scale the correction's time constant with how much of the pass is left.
