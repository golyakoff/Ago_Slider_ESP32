# Release 0.1.2

- OTA flow control: write responses on OTA_DATA/OTA_CONTROL are sent only after the data
  is written to flash (RSP_BY_APP), enabling the app to drop its per-chunk pacing delay.
- Prepared (long) write support on OTA_DATA for backward compatibility with app v0.1.0.

# Release 0.1.1

- Add Claude Code project skills (flash, release) and development docs.
- Test release for the end-to-end BLE OTA update path (v0.1.0 → v0.1.1).

# Release 0.1.0

- BLE GATT server: motion control (relative moves, homing to limit switches), per-axis
  configuration (microsteps, run/hold current, units, speed/accel, StealthChop, direction),
  power monitoring, limit switch notifications.
- BLE OTA firmware updates with image validation and bootloader rollback support.
