#ifndef __OTA_H__
#define __OTA_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// OTA Event Callbacks
// ============================================================================

/**
 * @brief Callback for OTA progress updates.
 * @param received Number of bytes received so far.
 * @param total    Total number of bytes expected for the firmware.
 */
typedef void (*ota_progress_cb_t)(size_t received, size_t total);

/**
 * @brief Callback for OTA status messages.
 * @param status   Null‑terminated status message.
 * @param is_error True if the status indicates an error, false otherwise.
 */
typedef void (*ota_status_cb_t)(const char* status, bool is_error);

/**
 * @brief Callback for OTA completion.
 * @param success True if the update completed successfully, false on failure.
 * @param message Null‑terminated message describing the outcome.
 */
typedef void (*ota_complete_cb_t)(bool success, const char* message);

// ============================================================================
// OTA Control Functions
// ============================================================================

/**
 * @brief Start an OTA update.
 *
 * Uses sequential-write mode: the target partition is erased incrementally
 * as data arrives, so this call returns quickly and does not block the
 * caller (typically the BLE task) on a full-partition erase.
 *
 * @param firmware_size          Expected total size of the new firmware.
 * @param on_ota_progress_cb     Callback for progress updates (may be NULL).
 * @param on_ota_status_cb       Callback for status messages (may be NULL).
 * @param on_ota_complete_cb     Callback for completion (may be NULL).
 * @return true if OTA started successfully, false otherwise.
 */
bool ota_begin(
    size_t firmware_size,
    ota_progress_cb_t on_ota_progress_cb,
    ota_status_cb_t on_ota_status_cb,
    ota_complete_cb_t on_ota_complete_cb
);

/**
 * @brief Write a chunk of firmware data to the OTA partition.
 * @param data   Pointer to the data buffer.
 * @param length Number of bytes to write.
 * @return true if the data was written successfully, false otherwise.
 */
bool ota_write(const uint8_t* data, size_t length);

/**
 * @brief Finalize the OTA update.
 *
 * Verifies that the expected number of bytes was received, lets
 * esp_ota_end() validate the image, sets the new boot partition and
 * schedules a reboot ~1.5 s later — the delay lets the BLE stack deliver
 * the write response / final notifications to the client before restart.
 *
 * @return true if the update was finalized successfully, false otherwise.
 */
bool ota_end(void);

/**
 * @brief Abort the ongoing OTA update and clean up resources.
 */
void ota_abort(void);

/**
 * @brief Check if an OTA update is currently in progress.
 * @return true if an OTA update is active, false otherwise.
 */
bool ota_is_in_progress(void);

/**
 * @brief Get the current running firmware version.
 *
 * Returns the version string from the app descriptor embedded in the
 * running image (set via PROJECT_VER in the top-level CMakeLists.txt).
 *
 * @return Pointer to the version string (valid for the app lifetime).
 */
const char* ota_get_current_version(void);

/**
 * @brief Confirm the currently running image after an OTA update.
 *
 * With CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, a freshly updated app boots
 * in the PENDING_VERIFY state; unless it is marked valid, the bootloader
 * rolls back to the previous image on the next reboot. Call this once at
 * startup after the critical subsystems have initialized successfully.
 */
void ota_confirm_running_image(void);

#ifdef __cplusplus
}
#endif

#endif // __OTA_H__
