# Release 0.1.4

- Hardware span calibration (CALIBRATE, 0xF006): the device seeks each end of an axis
  fast, retreats, re-seeks slowly and parks at the commanded offset, reporting the
  measured span. The endstop sensors are magnetic point triggers that blink for about
  4 ms on a passing carriage, so the stops now happen on-device — a BLE-driven loop
  could not catch them and would run the carriage into the hard stop.
- New POSITION characteristic (0xF005): commanded positions for all three axes, zeroed
  at the switch on homing and notified every 200 ms while moving.
- Endstop events no longer go missing: only the PCA9555 interrupt task reads the input
  registers, so nothing clears the INT latch behind its back.
- MOVE now takes 32-bit step counts per axis instead of 16-bit.

# Release 0.1.3

- Homing fixes: endstop state is never published from a failed I2C read (last known state
  is kept instead), so a homed axis no longer appears to leave its switch when another
  axis homes.
- Virtual limits are direction-aware: an axis sitting on its endstop can be driven away
  from it, and the 20 Hz forceStop/log flood while parked on a switch is gone.
- HOME status: a `requested` bit is cleared the moment its axis reaches the endstop, so
  it now means "still on its way"; a bit that survives the run marks an axis that never
  made it.
- Homing timeout raised from 30 s to 90 s — the X carriage needs more than 30 s to cross
  the full rail.

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
