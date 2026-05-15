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
bool ota_write(uint8_t* data, size_t length);

/**
 * @brief Finalize the OTA update, validate the firmware, and reboot.
 * @return true if the update was finalized successfully, false otherwise.
 */
bool ota_end(void);

/**
 * @brief Abort the ongoing OTA update and clean up resources.
 */
void ota_abort(void);

/**
 * @brief Get the current running firmware version.
 * @return Pointer to a statically allocated version string.
 */
const char* ota_get_current_version(void);

/**
 * @brief Validate the integrity of the received firmware.
 * @return true if the firmware is valid, false otherwise.
 */
bool ota_validate_firmware(void);

/**
 * @brief Check if an OTA update is currently in progress.
 * @return true if an OTA update is active, false otherwise.
 */
bool ota_is_in_progress(void);

/**
 * @brief Get the number of bytes received so far.
 * @return Number of bytes already written.
 */
size_t ota_get_bytes_received(void);

/**
 * @brief Get the total expected firmware size.
 * @return Total size of the firmware (bytes).
 */
size_t ota_get_total_size(void);

// ============================================================================
// OTA Event Emitters (for internal use)
// ============================================================================

/**
 * @brief Emit a status message via the registered status callback.
 * @param status   Status string.
 * @param is_error True if this is an error, false otherwise.
 */
void ota_emit_status(const char* status, bool is_error);

/**
 * @brief Emit a progress update via the registered progress callback.
 * @param received Bytes received so far.
 * @param total    Total expected bytes.
 */
void ota_emit_progress(size_t received, size_t total);

/**
 * @brief Emit a completion event via the registered completion callback.
 * @param success  True on success, false on failure.
 * @param message  Result message.
 */
void ota_emit_complete(bool success, const char* message);

#ifdef __cplusplus
}
#endif

#endif // __OTA_H__
