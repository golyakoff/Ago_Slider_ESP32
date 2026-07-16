---
name: flash
description: Build the AGO Slider firmware and flash it to the ESP32 over the serial port, optionally with monitor. Use when asked to build, flash, or monitor the firmware on the device.
---

# Build & flash the firmware

## Environment

`idf.py` is NOT on PATH by default. Activate the ESP-IDF 5.5.4 environment first —
in PowerShell (not Bash):

```powershell
. "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1"
```

This sets IDF_PATH (`C:\esp\v5.5.4\esp-idf`), the Python venv and puts `idf.py`,
`esptool.py` etc. on PATH. It must be re-run in every new shell invocation (shell
state does not persist between tool calls).

## Build

```powershell
Set-Location "C:\xMC\Ago_Slider_ESP32"; idf.py build
```

Local builds use the committed `sdkconfig` (NOT `sdkconfig.defaults` — that file plus
the `sdkconfig.debug`/`sdkconfig.release` overlays is only for CI).

## Flash

The device is on **COM4** (see `.vscode/settings.json`, `idf.portWin`):

```powershell
idf.py -p COM4 flash
```

## Monitor

`idf.py monitor` is interactive and will block a tool call. Do not run it yourself —
suggest the user run it in their own terminal (they can type `! idf.py -p COM4 monitor`
in the Claude Code prompt; exit with Ctrl+]). For a quick non-interactive log capture
you may instead read the port for a bounded time via esptool-less serial read, but
prefer delegating monitor to the user.

## Verification notes

- After an OTA-related change, remember the rollback flow: the freshly flashed image
  runs as PENDING_VERIFY until `ota_confirm_running_image()` marks it valid at the end
  of `app_main()`; a device that reboots twice into the old firmware means confirmation
  did not happen.
- Serial flashing writes all four images (bootloader/partition table/otadata/app);
  `idf.py flash` handles that automatically.
