#include "ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "OTA";

// -----------------------------------------------------------------------------
// Static state variables
// -----------------------------------------------------------------------------
static bool s_ota_in_progress = false;          ///< True if an OTA update is active
static size_t s_firmware_bytes_received = 0;    ///< Number of bytes already written
static size_t s_firmware_total_size = 0;        ///< Expected total firmware size
static ota_progress_cb_t s_progress_cb = NULL;  ///< Progress callback (optional)
static ota_status_cb_t s_status_cb = NULL;      ///< Status callback (optional)
static ota_complete_cb_t s_complete_cb = NULL;  ///< Completion callback (optional)
static esp_ota_handle_t s_ota_handle = 0;       ///< OTA operation handle
static const esp_partition_t *s_update_partition = NULL; ///< Target OTA partition

// -----------------------------------------------------------------------------
// Public API implementation
// -----------------------------------------------------------------------------

bool ota_begin(size_t firmware_size,
               ota_progress_cb_t on_ota_progress_cb,
               ota_status_cb_t on_ota_status_cb,
               ota_complete_cb_t on_ota_complete_cb)
{
    // Abort if another update is already running
    if (s_ota_in_progress) {
        ota_emit_status("OTA already in progress", true);
        return false;
    }

    // Determine the next OTA partition (e.g., OTA_0 or OTA_1)
    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_update_partition == NULL) {
        ota_emit_status("Failed to get next update partition", true);
        return false;
    }

    ESP_LOGI(TAG, "Starting OTA to partition: %s, size: %d bytes",
             s_update_partition->label, (int)firmware_size);

    // Begin the OTA operation
    esp_err_t err = esp_ota_begin(s_update_partition, firmware_size, &s_ota_handle);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "esp_ota_begin failed: %s", esp_err_to_name(err));
        ota_emit_status(msg, true);
        return false;
    }

    // Initialize state
    s_ota_in_progress = true;
    s_firmware_total_size = firmware_size;
    s_firmware_bytes_received = 0;
    s_progress_cb = on_ota_progress_cb;
    s_status_cb = on_ota_status_cb;
    s_complete_cb = on_ota_complete_cb;

    // Emit initial progress (0%)
    ota_emit_progress(0, firmware_size);
    return true;
}

bool ota_write(uint8_t* data, size_t length)
{
    // Check that OTA has been started
    if (!s_ota_in_progress) {
        ota_emit_status("OTA not started", true);
        return false;
    }
    // Validate input parameters
    if (data == NULL || length == 0) {
        ota_emit_status("Invalid data parameters", true);
        return false;
    }

    // Small delay to stabilize the system during data reception
    vTaskDelay(pdMS_TO_TICKS(1));

    // Write the chunk to the OTA partition
    esp_err_t err = esp_ota_write(s_ota_handle, data, length);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "esp_ota_write failed: %s", esp_err_to_name(err));
        ota_emit_status(msg, true);
        ota_abort();           // abort update on write error
        return false;
    }

    // Update counters and emit progress
    s_firmware_bytes_received += length;
    ota_emit_progress(s_firmware_bytes_received, s_firmware_total_size);
    return true;
}

bool ota_end(void)
{
    // Check that OTA is active
    if (!s_ota_in_progress) {
        ota_emit_status("OTA not started", true);
        return false;
    }

    // Finish the OTA operation
    esp_err_t err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "esp_ota_end failed: %s", esp_err_to_name(err));
        ota_emit_status(msg, true);
        ota_abort();
        return false;
    }

    // Validate the received firmware
    if (!ota_validate_firmware()) {
        ota_emit_status("Firmware validation failed", true);
        ota_abort();
        return false;
    }

    // Set the newly updated partition as the boot partition
    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        ota_emit_status(msg, true);
        return false;
    }

    // Emit final progress (100%) and completion
    ota_emit_progress(s_firmware_total_size, s_firmware_total_size);
    ota_emit_complete(true, "OTA completed successfully");

    s_ota_in_progress = false;

    // Reboot the device after a short delay
    ota_emit_status("Rebooting...", false);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return true;
}

void ota_abort(void)
{
    if (s_ota_in_progress) {
        // Abort the OTA operation if a handle exists
        if (s_ota_handle) {
            esp_ota_abort(s_ota_handle);
        }
        ota_emit_status("OTA update aborted", true);
        ota_emit_complete(false, "Update aborted");
    }

    // Reset all state variables
    s_ota_in_progress = false;
    s_firmware_bytes_received = 0;
    s_firmware_total_size = 0;
    s_ota_handle = 0;
    s_update_partition = NULL;
}

const char* ota_get_current_version(void)
{
    // Extract version string from the running application descriptor
    static char version[32];
    const esp_app_desc_t *app_desc = esp_app_get_description();
    strlcpy(version, app_desc->version, sizeof(version));
    return version;
}

bool ota_validate_firmware(void)
{
    // Basic validation: received size must match expected size
    if (s_firmware_bytes_received != s_firmware_total_size) {
        ota_emit_status("Firmware size mismatch", true);
        return false;
    }
    // Additional checks (e.g., CRC, signature) can be added here
    return true;
}

bool ota_is_in_progress(void)
{
    return s_ota_in_progress;
}

size_t ota_get_bytes_received(void)
{
    return s_firmware_bytes_received;
}

size_t ota_get_total_size(void)
{
    return s_firmware_total_size;
}

// -----------------------------------------------------------------------------
// Internal helpers for emitting events through the registered callbacks
// -----------------------------------------------------------------------------

void ota_emit_status(const char* status, bool is_error)
{
    if (s_status_cb) {
        s_status_cb(status, is_error);
    } else {
        // Default to ESP_LOG if no callback is provided
        if (is_error) {
            ESP_LOGE(TAG, "%s", status);
        } else {
            ESP_LOGI(TAG, "%s", status);
        }
    }
}

void ota_emit_progress(size_t received, size_t total)
{
    if (s_progress_cb) {
        s_progress_cb(received, total);
    }
}

void ota_emit_complete(bool success, const char* message)
{
    if (s_complete_cb) {
        s_complete_cb(success, message);
    }
}
