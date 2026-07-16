#include "ota.h"

#include <inttypes.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "OTA";

// -----------------------------------------------------------------------------
// Static state
// -----------------------------------------------------------------------------
static bool s_ota_in_progress = false;          ///< True if an OTA update is active
static size_t s_bytes_received = 0;             ///< Number of bytes already written
static size_t s_total_size = 0;                 ///< Expected total firmware size
static ota_progress_cb_t s_progress_cb = NULL;  ///< Progress callback (optional)
static ota_status_cb_t s_status_cb = NULL;      ///< Status callback (optional)
static ota_complete_cb_t s_complete_cb = NULL;  ///< Completion callback (optional)
static esp_ota_handle_t s_ota_handle = 0;       ///< OTA operation handle
static const esp_partition_t *s_update_partition = NULL; ///< Target OTA partition

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------

static void emit_status(const char* status, bool is_error)
{
    if (is_error) {
        ESP_LOGE(TAG, "%s", status);
    } else {
        ESP_LOGI(TAG, "%s", status);
    }
    if (s_status_cb) {
        s_status_cb(status, is_error);
    }
}

static void emit_progress(size_t received, size_t total)
{
    if (s_progress_cb) {
        s_progress_cb(received, total);
    }
}

static void emit_complete(bool success, const char* message)
{
    if (s_complete_cb) {
        s_complete_cb(success, message);
    }
}

static void emit_error(const char* prefix, esp_err_t err)
{
    char msg[80];
    snprintf(msg, sizeof(msg), "%s: %s", prefix, esp_err_to_name(err));
    emit_status(msg, true);
}

static void reset_state(void)
{
    s_ota_in_progress = false;
    s_bytes_received = 0;
    s_total_size = 0;
    s_ota_handle = 0;
    s_update_partition = NULL;
}

static void reboot_timer_cb(void* arg)
{
    ESP_LOGI(TAG, "Rebooting into the new firmware...");
    esp_restart();
}

/**
 * Schedule a reboot so the current (BLE) task can finish delivering the
 * write response to the client before the device restarts.
 */
static void schedule_reboot(uint32_t delay_ms)
{
    const esp_timer_create_args_t args = {
        .callback = reboot_timer_cb,
        .name = "ota_reboot",
    };
    esp_timer_handle_t timer;
    esp_err_t err = esp_timer_create(&args, &timer);
    if (err == ESP_OK) {
        err = esp_timer_start_once(timer, (uint64_t)delay_ms * 1000);
    }
    if (err != ESP_OK) {
        // Fallback: reboot immediately rather than staying on the old image
        ESP_LOGW(TAG, "Failed to schedule reboot (%s), restarting now", esp_err_to_name(err));
        esp_restart();
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool ota_begin(size_t firmware_size,
               ota_progress_cb_t on_ota_progress_cb,
               ota_status_cb_t on_ota_status_cb,
               ota_complete_cb_t on_ota_complete_cb)
{
    if (s_ota_in_progress) {
        emit_status("OTA already in progress", true);
        return false;
    }
    if (firmware_size == 0) {
        emit_status("Invalid firmware size", true);
        return false;
    }

    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_update_partition == NULL) {
        emit_status("Failed to get next update partition", true);
        return false;
    }
    if (firmware_size > s_update_partition->size) {
        emit_status("Firmware does not fit into the OTA partition", true);
        s_update_partition = NULL;
        return false;
    }

    ESP_LOGI(TAG, "Starting OTA to partition '%s' (0x%08" PRIx32 "), size: %u bytes",
             s_update_partition->label, s_update_partition->address,
             (unsigned)firmware_size);

    // Sequential-write mode erases flash incrementally as data arrives,
    // so ota_begin() does not block on a full-partition erase.
    esp_err_t err = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (err != ESP_OK) {
        emit_error("esp_ota_begin failed", err);
        s_update_partition = NULL;
        return false;
    }

    s_ota_in_progress = true;
    s_total_size = firmware_size;
    s_bytes_received = 0;
    s_progress_cb = on_ota_progress_cb;
    s_status_cb = on_ota_status_cb;
    s_complete_cb = on_ota_complete_cb;

    emit_progress(0, firmware_size);
    return true;
}

bool ota_write(const uint8_t* data, size_t length)
{
    if (!s_ota_in_progress) {
        emit_status("OTA not started", true);
        return false;
    }
    if (data == NULL || length == 0) {
        emit_status("Invalid data parameters", true);
        return false;
    }
    if (s_bytes_received + length > s_total_size) {
        emit_status("Received more data than announced firmware size", true);
        ota_abort();
        return false;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, data, length);
    if (err != ESP_OK) {
        emit_error("esp_ota_write failed", err);
        ota_abort();
        return false;
    }

    s_bytes_received += length;
    emit_progress(s_bytes_received, s_total_size);
    return true;
}

bool ota_end(void)
{
    if (!s_ota_in_progress) {
        emit_status("OTA not started", true);
        return false;
    }

    if (s_bytes_received != s_total_size) {
        emit_status("Firmware size mismatch", true);
        ota_abort();
        return false;
    }

    // esp_ota_end() validates the received image (magic byte, checksum,
    // secure boot signature when enabled) and releases the handle.
    esp_err_t err = esp_ota_end(s_ota_handle);
    s_ota_handle = 0;
    if (err != ESP_OK) {
        emit_error("esp_ota_end failed", err);
        emit_complete(false, "Image validation failed");
        reset_state();
        return false;
    }

    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        emit_error("esp_ota_set_boot_partition failed", err);
        emit_complete(false, "Failed to set boot partition");
        reset_state();
        return false;
    }

    emit_progress(s_total_size, s_total_size);
    emit_complete(true, "OTA completed successfully");
    emit_status("Rebooting...", false);
    reset_state();

    schedule_reboot(1500);
    return true;
}

void ota_abort(void)
{
    if (s_ota_in_progress) {
        if (s_ota_handle) {
            esp_ota_abort(s_ota_handle);
        }
        emit_status("OTA update aborted", true);
        emit_complete(false, "Update aborted");
    }
    reset_state();
}

bool ota_is_in_progress(void)
{
    return s_ota_in_progress;
}

const char* ota_get_current_version(void)
{
    // The app descriptor lives in the memory-mapped app image,
    // so the pointer stays valid for the whole app lifetime.
    return esp_app_get_description()->version;
}

void ota_confirm_running_image(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;

    if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
        return;
    }

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "New firmware confirmed valid (rollback cancelled)");
        } else {
            ESP_LOGE(TAG, "Failed to confirm firmware: %s", esp_err_to_name(err));
        }
    }
}
