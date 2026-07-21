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

## Focus scenario: C/B lag at the end of a pass — ADDRESSED, awaiting hardware confirmation

Symptom (60 s pass, 770 mm, subject 27 cm): tracking held to ~0.05 deg for ~95% of the run,
then C fell ~49 pulses (0.65 deg) behind over X's ~1.5 s deceleration. Visible only in the log,
not reported on video, but a long lens or a butt-cut would show it.

Mechanism (now understood): the `cmd` in the log is the commanded speed CAP, not the actual
speed. As X decelerates the tracked axis's ACTUAL speed lags the shrinking cap because of its
own deceleration ramp, so it arrives short; the fixed 3 s correction time constant is far too
slow to close the gap in the ~1 s of deceleration left. Nothing "froze" — the axis just ran
slower than the cap while both were falling.

Fix applied (firmware, scenario_focus.cpp, not yet hardware-confirmed):
- Tracking acceleration for C and B raised from `peak/ramp_s` (~1.5 s to reach speed) to
  `peak/0.3 s`. The tracking is closed-loop on X's real speed, so a brisk acceleration only
  tightens how closely the axis holds the commanded speed; it does not over-lead at ramp-in
  because the command already follows X.
- Correction time constant scales down with the time remaining, floored at 1 s, so the small
  residual is driven out smoothly (~0.65 deg/s, invisible) before X stops, instead of being
  frozen in by the stop. The ease-out is kept — the user rejected both a hard forceStop (jolt +
  step-loss risk on the loaded X rail) and a post-stop settle phase (pads the shot).

If a residual remains after this, the next lever is a post-X-stop settle explicitly bounded in
time, but only if the user accepts a fraction of a second of post-roll.
