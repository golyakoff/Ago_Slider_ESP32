#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_device.h"

#include "ble.h"


#define TAG "BLE_SRV"

// ----------------------------- Attributes Indices ---------------------------------
enum {
    IDX_SVC,                             // Service declaration

    // Motor control (non-notifiable)
    IDX_CHAR_MOT_EN,                     // MOT_EN declaration
    IDX_CHAR_VAL_MOT_EN,                 // MOT_EN value

    // Move command (non-notifiable)
    IDX_CHAR_MOVE,                       // MOVE declaration
    IDX_CHAR_VAL_MOVE,                   // MOVE value

    // Endstop sensors (notifiable)
    IDX_CHAR_LIMIT,                      // LIMIT declaration
    IDX_CHAR_VAL_LIMIT,                  // LIMIT value
    IDX_CHAR_LIMIT_CFG,                  // LIMIT CCCD

    // Homing (notifiable)
    IDX_CHAR_HOME,                       // HOME declaration
    IDX_CHAR_VAL_HOME,                   // HOME value (read/write)
    IDX_CHAR_HOME_CFG,                   // HOME CCCD

    // Battery level (notifiable)
    IDX_CHAR_BATT_LEVEL,                 // Battery level declaration
    IDX_CHAR_VAL_BATT_LEVEL,             // Battery level value (0-255)
    IDX_CHAR_BATT_LEVEL_CFG,             // Battery level CCCD

     // Power monitoring binary (notifiable)
    IDX_CHAR_PWR_INFO,                   // Power info (binary) declaration
    IDX_CHAR_VAL_PWR_INFO,               // Power info value (12 bytes)
    IDX_CHAR_PWR_INFO_CFG,               // Power info CCCD

    // Power monitoring human-readable (notifiable)
    IDX_CHAR_PWR_INFO_STR,               // Power info string declaration
    IDX_CHAR_VAL_PWR_INFO_STR,           // Power info string value (max 40 chars)
    IDX_CHAR_PWR_INFO_STR_CFG,           // Power info string CCCD

    // ----- Configuration -----

    // Microsteps (non-notifiable)
    IDX_CHAR_MICROSTEPS,                 // Microsteps declaration (0..7 encoding: 256,1,2,4,8,16,32,64)
    IDX_CHAR_VAL_MICROSTEPS,             // Microsteps value (3 bytes - one per axis)

    // Run current (non-notifiable)
    IDX_CHAR_RUN_CURRENT,                // Run current declaration
    IDX_CHAR_VAL_RUN_CURRENT,            // Run current value (6 bytes, 3 uint16, mA)

    // Hold current (non-notifiable)
    IDX_CHAR_HOLD_CURRENT,               // Hold current declaration
    IDX_CHAR_VAL_HOLD_CURRENT,           // Hold current value (6 bytes, 3 uint16, mA)

    // Axis unit (non-notifiable)
    IDX_CHAR_AXIS_UNIT,                  // Axis unit declaration
    IDX_CHAR_VAL_AXIS_UNIT,              // Axis unit value (1 byte, bits: bit0=X, bit1=C, bit2=B; 0=mm,1=degrees)

    // Units per step (non-notifiable)
    IDX_CHAR_UNITS_PER_STEP,             // Units per step declaration
    IDX_CHAR_VAL_UNITS_PER_STEP,         // Units per step value (12 bytes, 3 floats little-endian)

    // Axis speed (non-notifiable)
    IDX_CHAR_AXIS_SPEED,                 // Axis speed declaration
    IDX_CHAR_VAL_AXIS_SPEED,             // Axis speed value (6 bytes, 3 uint16 steps per second)

    // Axis acceleration (non-notifiable)
    IDX_CHAR_AXIS_ACCEL,                  // Axis acceleration declaration
    IDX_CHAR_VAL_AXIS_ACCEL,              // Axis acceleration value (6 bytes, 3 uint16)

    // Virtual limit (non-notifiable)
    IDX_CHAR_VIRTUAL_LIMIT,              // Virtual limit enable declaration
    IDX_CHAR_VAL_VIRTUAL_LIMIT,          // Virtual limit enable value (1 byte, bits: bit0=X, bit1=C, bit2=B)

    // Stealthchop (non-notifiable)
    IDX_CHAR_STEALTHCHOP,                // Stealthchop declaration
    IDX_CHAR_VAL_STEALTHCHOP,            // Stealthchop value (1 byte, bits per axis)

    // Invert direction (non-notifiable)
    IDX_CHAR_INVERT_DIR,                 // Invert direction declaration
    IDX_CHAR_VAL_INVERT_DIR,             // Invert direction value (1 byte, bits per axis)

    // ----- OTA -----
    IDX_CHAR_VERSION,                   // Version declaration
    IDX_CHAR_VAL_VERSION,               // Version value (read + notify)
    IDX_CHAR_VERSION_CFG,               // CCCD for version notifications

    IDX_CHAR_OTA_CONTROL,               // OTA control declaration
    IDX_CHAR_VAL_OTA_CONTROL,           // OTA control value (write only)

    IDX_CHAR_OTA_DATA,                  // OTA data declaration
    IDX_CHAR_VAL_OTA_DATA,              // OTA data value (write only)

    // ----- TOTAL ----- 
    HRS_IDX_NB                           // Total attributes count
};

// ----------------------------- UUIDs ----------------------------------------------
#define BLE_SERVICE_UUID            0xFE95

#define BLE_MOT_EN_CHAR_UUID        0xF001  // Enable all motors, 1 byte.
                                            // Write : 1 to enable motors, 0 to disable.
                                            // Read  : current state (1 = enabled, 0 = disabled).
                                            // No notification – this is a fast, almost synchronous command.

#define BLE_AXIS

#define BLE_HOME_CHAR_UUID          0xF002  // Home command and status, 1 byte.
                                            // Write : command to home specified axes.
                                            //        Bits (high nibble, bits 4-6):
                                            //          bit 4 (0b00010000): home X
                                            //          bit 5 (0b00100000): home C
                                            //          bit 6 (0b01000000): home B
                                            //        Example: 0x70 = home all axes (bits 4,5,6 set).
                                            //        Lower nibble is ignored on write.
                                            //        When an axis completes homing, its command bit is cleared.
                                            // Read  : current homing status.
                                            //        High nibble (bits 4-6): axes still scheduled for homing.
                                            //        Low nibble (bits 0-2): axes already homed.
                                            //          bit 0 (0b00000001): X homed
                                            //          bit 1 (0b00000010): C homed
                                            //          bit 2 (0b00000100): B homed
                                            // Notify: sent whenever homing status changes.
                                            //        Example sequence (homing order X → C → B):
                                            //          0x70 (0b01110000) – all three scheduled (bits 4-6 = 111), none homed.
                                            //          0x61 (0b01100001) – X homed (low bit 0 =1), C and B still scheduled (bits 5,6 = 1).
                                            //          0x43 (0b01000011) – X,C homed (low bits 0,1 = 1), B still scheduled (bit 6 = 1).
                                            //          0x07 (0b00000111) – all homed (low bits 0,1,2 = 1), no scheduled bits.

#define BLE_LIMIT_CHAR_UUID         0xF003  // Endstop sensor status, 1 byte.
                                            // Read  : limit switch states for each axis.
                                            //        Bits (low nibble, bits 0-2):
                                            //          bit 0 (0x01): X limit reached
                                            //          bit 1 (0x02): C limit reached
                                            //          bit 2 (0x04): B limit reached

#define BLE_MOVE_CHAR_UUID          0xF004  // Move command: relative distance for each axis, 6 bytes (3 x int16).
                                            // Write : signed distance in steps (or mm/deg with fixed point).
                                            //        Bytes 0-1: X (int16, little‑endian)
                                            //        Bytes 2-3: C (int16, little‑endian)
                                            //        Bytes 4-5: B (int16, little‑endian)
                                            //        Positive = forward/clockwise, negative = backward/counter‑clockwise.
                                            // Read  : not used (write‑only). Returns zeroes.
                                            // Notify: not used – the command is executed immediately.
                                            // Note: Speed and acceleration are pre‑configured or set via other characteristics.

#define BLE_BATT_LEVEL_CHAR_UUID    0xF020  // Battery level, 1 byte.
                                            // Read  : current battery level (0-255).
                                            //        Linear mapping: 0 corresponds to 18.0 V,
                                            //        255 corresponds to 21.5 V (5‑cell Li‑ion fully charged).
                                            // Notify: sent when battery level changes (if notification enabled).
                                            // Note : Client can enable notifications via CCCD.
                                            //        Use simple scaling: percent = (level * 100) / 255.

#define BLE_PWR_INFO_CHAR_UUID      0xF021  // Power measurement, 12 bytes (3 x float, little‑endian).
                                            // Read  : current voltage, current, and power.
                                            //        Bytes 0-3: voltage (float, volts)
                                            //        Bytes 4-7: current (float, amperes)
                                            //        Bytes 8-11: power (float, watts) = voltage * current
                                            // Notify: sent periodically (every 2 s) if enabled.
                                            // Note : Suitable for client‑side numeric processing (e.g., graphs).

#define BLE_PWR_INFO_STR_CHAR_UUID  0xF022  // Power info as formatted string, up to 40 bytes.
                                            // Read  : human‑readable string with three values.
                                            //        Format: "XX.XXV Y.YYYA Z.ZZW", e.g. "21.48V 0.082A 1.76W"
                                            //        Voltage: 2 decimal places, current: 3, power: 2.
                                            // Notify: sent simultaneously with BLE_PWR_INFO_CHAR_UUID.
                                            // Note : Convenient for debug terminals or simple displays.

                                            // Configuration characteristics

#define BLE_MICROSTEPS_CHAR_UUID    0xF030  // Number of microsteps: 
                                            // 3 bytes: 1 byte per axis - 0=256,1=1,2=2,4,8,16,32,64,128.
                                            // Write : 1 byte value: 
                                            //        Byte 0: X
                                            //        Byte 1: C
                                            //        Byte 2: B
                                            // Read  : the same values as in write sections.

#define BLE_RUN_CURRENT_CHAR_UUID   0xF031  // Run current for each axis, 6 bytes (3 x uint16).
                                            //      Current in mA, little‑endian per axis.
                                            // Write : 3 uint16 in order: X, C, B.
                                            // Read  : same as written.

#define BLE_HOLD_CURRENT_CHAR_UUID  0xF032  // Hold current for each axis, 6 bytes (3 x uint16).
                                            //      Current in mA, little‑endian per axis.
                                            // Write : 3 uint16 in order: X, C, B.
                                            // Read  : same as written.                                            

#define BLE_AXIS_UNIT_CHAR_UUID     0xF033  // Axis units, 3 bits in 1 byte:
                                            //      0 = "mm" for linear axis,
                                            //      1 = "degrees" for radial axis (bits)
                                            // Write : Bits (low nibble, bits 0-2):
                                            //      bit 0 (0x01): X axis unit
                                            //      bit 1 (0x02): C axis unit
                                            //      bit 2 (0x04): B axis unit
                                            // Read  : the same values as in write sections.

#define BLE_UNITS_PER_STEP_CHAR_UUID 0xF034 // Units per single step. 12 bytes (3 floats).
                                            //      A float (4 bytes) per axis.
                                            //          for linear axis "mm/step"
                                            //          for radial axis "deg/step"
                                            // Write : 3 floats in the following order: X, C, B.
                                            // Read  : the same values as in write sections.

#define BLE_AXIS_SPEED_CHAR_UUID    0xF035  // Axis speed for each axis. 6 bytes (3 uint16).
                                            //      Steps per second value for each axis.
                                            // Write : 3 uint16 in the following order: X, C, B.
                                            // Read  : the same values as in write sections.

#define BLE_AXIS_ACCEL_CHAR_UUID    0xF036  // Axis acceleration for each axis. 6 bytes (3 uint16).
                                            //      Steps per second value for each axis.
                                            // Write : 3 uint16 in the following order: X, C, B.
                                            // Read  : the same values as in write sections.

#define BLE_VIRTUAL_LIMIT_CHAR_UUID 0xF037  // Virtual limits enable. 1 byte (bits).
                                            //      Set bits to enable virtual limits per axis.
                                            // Write : Bits (low nibble, bits 0-2):
                                            //      bit 0 (0x01): X axis virtual limit
                                            //      bit 1 (0x02): C axis virtual limit
                                            //      bit 2 (0x04): B axis virtual limit
                                            // Read  : the same values as in write sections.

#define BLE_STEALTHCHOP_CHAR_UUID   0xF038  // StealthChop enable flags, 1 byte (bits).
                                            //      bit0: X axis, bit1: C axis, bit2: B axis.
                                            // Write : 1 byte with bits.
                                            // Read  : same as written.

#define BLE_INVERT_DIR_CHAR_UUID    0xF039  // Invert direction flags, 1 byte (bits).
                                            //      bit0: X axis, bit1: C axis, bit2: B axis.
                                            // Write : 1 byte with bits.
                                            // Read  : same as written.

#define BLE_VERSION_CHAR_UUID       0xF090  // Firmware version characteristic (read + notify)
                                            // String format, e.g., "1.0.0"
                                            // Write : 32 bytes of chars.
                                            // Read  : Same as written.
                                            // Notify: Sent new version after upgrade

#define BLE_OTA_CONTROL_CHAR_UUID   0xF091  // OTA control characteristic 5 bytes (write only).
                                            // Command bytes: 0x01 = start, 0x02 = end, 0x03 = abort
                                            // For start command, 4 following bytes contain total firmware size (uint32_t little‑endian)

#define BLE_OTA_DATA_CHAR_UUID      0xF092  // OTA data characteristic 512 bytes (write only).
                                            // Raw firmware data chunks (max MTU-3)

// ----------------------------- Constants ------------------------------------------
#define CHAR_DECLARATION_SIZE 1

// ----------------------------- CHARS ----------------------------------------------
#define CHAR_PROP_READ_WRITE_NOTIFY (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY)
#define CHAR_PROP_READ_NOTIFY       (ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY)
#define CHAR_PROP_WRITE             (ESP_GATT_CHAR_PROP_BIT_WRITE)

// CONFIGURATION CONSTANTS (as variables for address taking)
static const uint16_t svc_uuid = BLE_SERVICE_UUID;
static const uint16_t mot_en_char_uuid = BLE_MOT_EN_CHAR_UUID;
static const uint16_t home_char_uuid = BLE_HOME_CHAR_UUID;
static const uint16_t limit_char_uuid = BLE_LIMIT_CHAR_UUID;
static const uint16_t move_char_uuid = BLE_MOVE_CHAR_UUID;

static const uint16_t pwr_info_str_char_uuid = BLE_PWR_INFO_STR_CHAR_UUID;
static const uint16_t pwr_info_char_uuid = BLE_PWR_INFO_CHAR_UUID;
static const uint16_t batt_level_char_uuid = BLE_BATT_LEVEL_CHAR_UUID;

static const uint16_t microsteps_char_uuid = BLE_MICROSTEPS_CHAR_UUID;
static const uint16_t axis_unit_char_uuid = BLE_AXIS_UNIT_CHAR_UUID;
static const uint16_t run_current_char_uuid = BLE_RUN_CURRENT_CHAR_UUID;
static const uint16_t hold_current_char_uuid = BLE_HOLD_CURRENT_CHAR_UUID;
static const uint16_t units_per_step_char_uuid = BLE_UNITS_PER_STEP_CHAR_UUID;
static const uint16_t axis_speed_char_uuid = BLE_AXIS_SPEED_CHAR_UUID;
static const uint16_t axis_accel_char_uuid = BLE_AXIS_ACCEL_CHAR_UUID;
static const uint16_t virtual_limit_char_uuid = BLE_VIRTUAL_LIMIT_CHAR_UUID;
static const uint16_t stealthchop_char_uuid = BLE_STEALTHCHOP_CHAR_UUID;
static const uint16_t invert_dir_char_uuid = BLE_INVERT_DIR_CHAR_UUID;
static const uint16_t version_char_uuid_str = BLE_VERSION_CHAR_UUID;
static const uint16_t ota_control_char_uuid_str = BLE_OTA_CONTROL_CHAR_UUID;
static const uint16_t ota_data_char_uuid_str = BLE_OTA_DATA_CHAR_UUID;

static const uint8_t char_prop_rwn = CHAR_PROP_READ_WRITE_NOTIFY;
static const uint8_t char_prop_rn = CHAR_PROP_READ_NOTIFY;
static const uint8_t char_prop_w = CHAR_PROP_WRITE;
static const uint8_t char_declaration_size = 1;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

// Default values for characteristics
static const uint8_t move_default_val[12] = {0};
static const uint8_t limit_default_val[1] = {0};

static uint8_t microsteps_val[3] =      {16, 16, 16};                        // 1/16 for all axis
static uint8_t run_current_val[6] =     {900 & 0xFF, (900 >> 8) & 0xFF,     // 900 mA for X
                                         700 & 0xFF, (700 >> 8) & 0xFF,     // 700 mA for C
                                         900 & 0xFF, (900 >> 8) & 0xFF};    // 900 mA for B
static uint8_t hold_current_val[6] =    {450 & 0xFF, (450 >> 8) & 0xFF,     // 450 mA for X
                                         350 & 0xFF, (350 >> 8) & 0xFF,     // 350 mA for C
                                         450 & 0xFF, (450 >> 8) & 0xFF};    // 450 mA for B
static uint8_t axis_unit_val[1] =       {0x06};                             // Bits: X=0(mm), C=1(deg), B=1(deg)
static uint8_t units_per_step_val[12] = {0x9A, 0x8F, 0x44, 0x3E,            // X axis: 0.19195 mm/step = (360/200)*(38,39/360) = 38.39 / 200
                                         0x9A, 0x99, 0x59, 0x3E,            // C axis: 0.2125 deg/step = (360/200) * gear_ratio(17:144)
                                         0xE7, 0xD7, 0x95, 0x3E};           // B axis: 0.292682927 deg/step = (360/200) * gear_ratio(20:123)
static uint8_t axis_speed_val[6] =      {1000 & 0xFF, (1000 >> 8) & 0xFF,   // 1000 steps/sec for X
                                         1000 & 0xFF, (1000 >> 8) & 0xFF,   // 1000 steps/sec for C
                                         1000 & 0xFF, (1000 >> 8) & 0xFF};  // 1000 steps/sec for B
static uint8_t axis_accel_val[6] =       {1000 & 0xFF, (1000 >> 8) & 0xFF,   // 1000 steps/sec for X
                                         1000 & 0xFF, (1000 >> 8) & 0xFF,   // 1000 steps/sec for C
                                         1000 & 0xFF, (1000 >> 8) & 0xFF};  // 1000 steps/sec for B
static uint8_t virtual_limit_val[1] =   {0x05};                             // Bits: X (0x01) and B (0x04) enable virtual limit;
                                                                            //       C axis (0x02) disabled because it can rotate
                                                                            //       continuously beyond 360°
static uint8_t stealthchop_val[1] =     {0x07};                             // StealthChop enabled for all axes
static uint8_t invert_dir_val[1] =      {0x00};                             // No inverted direction

static char version_val[32] = "0.0.1";          // firmware version string (max 31 chars)
static uint8_t ota_control_val[5] = {0};        // up to 5 bytes (command + 32-bit size)
static uint8_t ota_data_val[512] = {0};         // buffer for OTA data chunks (adjust size if needed)

// Reassembly buffer for prepared (long) writes to OTA_DATA — used when the
// client's chunk size exceeds MTU-3 and Android falls back to ATT
// Prepare/Execute Write. Flashed on ESP_GATTS_EXEC_WRITE_EVT.
static uint8_t  ota_prep_buf[512];
static uint16_t ota_prep_len = 0;

// Default callbacks
static ble_start_cb_t s_start_cb = NULL;
static ble_connect_cb_t s_connect_cb = NULL;
static ble_disconnect_cb_t s_disconnect_cb = NULL;

static ble_limit_read_cb_t s_limit_read_cb = NULL;
static ble_mot_en_cb_t s_mot_en_cb = NULL;
static ble_home_cmd_cb_t s_home_cmd_cb = NULL;
static ble_move_cb_t s_move_cb = NULL;

static ble_microsteps_cb_t s_microsteps_cb = NULL;
static ble_run_current_cb_t s_run_current_cb = NULL;
static ble_hold_current_cb_t s_hold_current_cb = NULL;
static ble_axis_unit_cb_t s_axis_unit_cb = NULL;
static ble_units_per_step_cb_t s_units_per_step_cb = NULL;
static ble_axis_speed_cb_t s_axis_speed_cb = NULL;
static ble_axis_accel_cb_t s_axis_accel_cb = NULL;
static ble_virtual_limit_cb_t s_virtual_limit_cb = NULL;
static ble_stealthchop_cb_t s_stealthchop_cb = NULL;
static ble_invert_dir_cb_t s_invert_dir_cb = NULL;

static ble_version_read_cb_t s_version_read_cb = NULL;
static ble_ota_control_cb_t s_ota_control_cb = NULL;
static ble_ota_data_cb_t s_ota_data_cb = NULL;

// CCCD enable states
static bool limit_notify_enabled = false;
static bool home_notify_enabled = false;
static bool batt_level_notify = false;
static bool pwr_info_notify = false;
static bool pwr_info_str_notify = false;
static bool version_notify_enabled = false;

// ----------------------------- Attribute Table ------------------------------------
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Service Declaration
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(svc_uuid), (uint8_t *)&svc_uuid}
    },

    // ---- Notifiable characteristics (with CCCD) ----

    // LIMIT Characteristic Declaration
    [IDX_CHAR_LIMIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         char_declaration_size, char_declaration_size, (uint8_t *)&char_prop_rn}
    },

    // LIMIT Characteristic Value
    [IDX_CHAR_VAL_LIMIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&limit_char_uuid, ESP_GATT_PERM_READ,
         1, sizeof(limit_default_val), (uint8_t *)limit_default_val}
    },

    // LIMIT CCCD
    [IDX_CHAR_LIMIT_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // HOME Characteristic Declaration
    [IDX_CHAR_HOME] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // HOME Characteristic Value
    [IDX_CHAR_VAL_HOME] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&home_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, (uint8_t *)"\x00"}  // start status = 0
    },

    // HOME CCCD
    [IDX_CHAR_HOME_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // ---- Non-notifiable characteristics ----
    
    // MOT_EN Characteristic Declaration
    [IDX_CHAR_MOT_EN] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // MOT_EN Characteristic Value
    [IDX_CHAR_VAL_MOT_EN] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&mot_en_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, (uint8_t *)"\x00"}  // default disabled
    },

    // MOVE Characteristic Declaration
    [IDX_CHAR_MOVE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         char_declaration_size, char_declaration_size, (uint8_t *)&char_prop_w}
    },

    // MOVE Characteristic Value
    [IDX_CHAR_VAL_MOVE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&move_char_uuid, ESP_GATT_PERM_WRITE,
         12, 12, (uint8_t *)move_default_val}
    },

    // BATT_LEVEL Characteristic Declaration
    [IDX_CHAR_BATT_LEVEL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rn}
    },

    // BATT_LEVEL Characteristic Value
    [IDX_CHAR_VAL_BATT_LEVEL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&batt_level_char_uuid, ESP_GATT_PERM_READ,
         1, 1, (uint8_t *)"\x00"}
    },
    
    // BATT_LEVEL CCCD
    [IDX_CHAR_BATT_LEVEL_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // PWR_INFO (binary) Characteristic Declaration
    [IDX_CHAR_PWR_INFO] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rn}
    },
    
    // PWR_INFO (binary) Characteristic Value
    [IDX_CHAR_VAL_PWR_INFO] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&pwr_info_char_uuid, ESP_GATT_PERM_READ,
         12, 12, (uint8_t *)&(uint8_t[12]){0}} // 3 floats = 12 bytes
    },
    
    // PWR_INFO (binary) CCCD
    [IDX_CHAR_PWR_INFO_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // PWR_INFO (human-readble) Characteristic Declaration
    [IDX_CHAR_PWR_INFO_STR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rn}
    },

    // PWR_INFO (human-readble) Characteristic Value
    [IDX_CHAR_VAL_PWR_INFO_STR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&pwr_info_str_char_uuid, ESP_GATT_PERM_READ,
         40, 40, (uint8_t *)""} // max 40 chars
    },

    // PWR_INFO (human-readble) CCCD
    [IDX_CHAR_PWR_INFO_STR_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // ----- Configuration (non-notifiable) -----
    
    // MICROSTEPS Characteristic Declaration
    [IDX_CHAR_MICROSTEPS] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // MICROSTEPS Characteristic Value
    [IDX_CHAR_VAL_MICROSTEPS] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&microsteps_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         3, 3, microsteps_val}
    },

     // RUN_CURRENT Characteristic Declaration
    [IDX_CHAR_RUN_CURRENT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // RUN_CURRENT Characteristic Value
    [IDX_CHAR_VAL_RUN_CURRENT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&run_current_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         6, 6, run_current_val}
    },

    // HOLD_CURRENT Characteristic Declaration
    [IDX_CHAR_HOLD_CURRENT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // HOLD_CURRENT Characteristic Value
    [IDX_CHAR_VAL_HOLD_CURRENT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&hold_current_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         6, 6, hold_current_val}
    },

    // AXIS_UNIT Characteristic Declaration
    [IDX_CHAR_AXIS_UNIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // AXIS_UNIT Characteristic Value
    [IDX_CHAR_VAL_AXIS_UNIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&axis_unit_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, axis_unit_val}
    },
    
    // UNITS_PER_STEP Characteristic Declaration
    [IDX_CHAR_UNITS_PER_STEP] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // UNITS_PER_STEP Characteristic Value
    [IDX_CHAR_VAL_UNITS_PER_STEP] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&units_per_step_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         12, 12, units_per_step_val}
    },

    // AXIS_SPEED Characteristic Declaration
    [IDX_CHAR_AXIS_SPEED] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // AXIS_SPEED Characteristic Value
    [IDX_CHAR_VAL_AXIS_SPEED] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&axis_speed_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         6, 6, axis_speed_val}
    },

    // AXIS_ACCEL Characteristic Declaration
    [IDX_CHAR_AXIS_ACCEL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // AXIS_ACCEL Characteristic Value
    [IDX_CHAR_VAL_AXIS_ACCEL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&axis_accel_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         6, 6, axis_accel_val}
    },
    // VIRTUAL_LIMIT Characteristic Declaration
    [IDX_CHAR_VIRTUAL_LIMIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // VIRTUAL_LIMIT Characteristic Value
    [IDX_CHAR_VAL_VIRTUAL_LIMIT] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&virtual_limit_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, virtual_limit_val}
    },

        // STEALTHCHOP Characteristic Declaration
    [IDX_CHAR_STEALTHCHOP] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // STEALTHCHOP Characteristic Value
    [IDX_CHAR_VAL_STEALTHCHOP] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&stealthchop_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, stealthchop_val}
    },

    // INVERT_DIR Characteristic Declaration
    [IDX_CHAR_INVERT_DIR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rwn}
    },

    // INVERT_DIR Characteristic Value
    [IDX_CHAR_VAL_INVERT_DIR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&invert_dir_char_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         1, 1, invert_dir_val}
    },

    // ----- OTA -----
    
    // Version Characteristic Declaration
    [IDX_CHAR_VERSION] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_rn}
    },

    // Version Characteristic Value
    [IDX_CHAR_VAL_VERSION] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&version_char_uuid_str, ESP_GATT_PERM_READ,
         sizeof(version_val), sizeof(version_val), (uint8_t *)version_val}
    },

    // Version Characteristic CCCD
    [IDX_CHAR_VERSION_CFG] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), 2, (uint8_t *)"\x00\x00"}
    },

    // OTA Control Characteristic Declaration
    [IDX_CHAR_OTA_CONTROL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_w}
    },

    // OTA Control Characteristic Value.
    // RSP_BY_APP: the ATT write response is sent by the app only after the
    // command has been processed (for 0x02 "end" that includes image
    // validation and setting the boot partition), so the client's write
    // completion doubles as an acknowledgement.
    [IDX_CHAR_VAL_OTA_CONTROL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16, (uint8_t *)&ota_control_char_uuid_str, ESP_GATT_PERM_WRITE,
         5, 5, ota_control_val}
    },

    // OTA Data Characteristic Declaration
    [IDX_CHAR_OTA_DATA] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_w}
    },

    // OTA Data Characteristic Value.
    // RSP_BY_APP: the response is sent only after esp_ota_write() has
    // finished, giving the client per-chunk flow control (no artificial
    // pacing delays needed on the phone side).
    [IDX_CHAR_VAL_OTA_DATA] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16, (uint8_t *)&ota_data_char_uuid_str, ESP_GATT_PERM_WRITE,
         512, 512, ota_data_val}
    },
};

// ----------------------------- Handles Storage ------------------------------------
static uint16_t handle_table[HRS_IDX_NB];
static uint16_t conn_id = 0;
static esp_gatt_if_t s_gatts_if = 0;
static bool connected = false;
static bool attr_table_ready = false;  // handle_table is only valid once the table is created

// ----------------------------- Forward Declarations -------------------------------
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void start_advertising(void);

// ----------------------------- GATTS Event Handler --------------------------------
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            s_gatts_if = gatts_if;
            ESP_LOGI(TAG, "GATTS registered, app_id=%d", param->reg.app_id);
            // Create attribute table
            esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Create attr table failed: %s", esp_err_to_name(ret));
            }
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Create attr table failed, status=0x%x", param->add_attr_tab.status);
                break;
            }
            if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
                ESP_LOGE(TAG, "Attr table num_handle mismatch: got %d, expected %d",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
                break;
            }
            // Store handles
            memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
            attr_table_ready = true;
            // Seed the version now, before any client can read it: the attribute still holds
            // the build-time placeholder, and the READ_EVT handler below refreshes it only
            // after the stack has already auto-responded — so without this the first read
            // after a reboot would report the placeholder instead of the running version.
            if (s_version_read_cb) {
                const char *ver = s_version_read_cb();
                esp_ble_gatts_set_attr_value(
                    handle_table[IDX_CHAR_VAL_VERSION], strlen(ver), (uint8_t *)ver);
            }
            // Start service
            esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            // Start advertising
            start_advertising();
            break;

        case ESP_GATTS_CONNECT_EVT:
            conn_id = param->connect.conn_id;
            connected = true;
            ESP_LOGI(TAG, "Client connected, conn_id=%d", conn_id);
            if (s_connect_cb) s_connect_cb();
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            connected = false;
            ESP_LOGI(TAG, "Client disconnected, restart advertising");
            if (s_disconnect_cb) s_disconnect_cb();
            start_advertising();
            break;

        case ESP_GATTS_WRITE_EVT:
        {
            uint16_t handle = param->write.handle;
            // Handle MOT_EN write
            if (handle == handle_table[IDX_CHAR_VAL_MOT_EN] && param->write.len >= 1 && s_mot_en_cb) {
                uint8_t en = param->write.value[0];
                s_mot_en_cb(en);
                ESP_LOGI(TAG, "MOT_EN: %d", en);
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle HOME command write
            else if (handle == handle_table[IDX_CHAR_VAL_HOME] && param->write.len >= 1 && s_home_cmd_cb) {
                uint8_t cmd = param->write.value[0];
                bool home_x = (cmd & 0x10) != 0;
                bool home_c = (cmd & 0x20) != 0;
                bool home_b = (cmd & 0x40) != 0;
                s_home_cmd_cb(home_x, home_c, home_b);
                ESP_LOGI(TAG, "HOME command: X=%d, C=%d, B=%d", home_x, home_c, home_b);
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle MOVE characteristic write
            else if (handle == handle_table[IDX_CHAR_VAL_MOVE]) {
                if (param->write.len == 12 && s_move_cb) {
                    int32_t x =   (int32_t)param->write.value[0] |
                                 ((int32_t)param->write.value[1]  <<  8) |
                                 ((int32_t)param->write.value[2]  << 16) |
                                 ((int32_t)param->write.value[3]  << 24);
                    int32_t c =   (int32_t)param->write.value[4] |
                                 ((int32_t)param->write.value[5]  <<  8) |
                                 ((int32_t)param->write.value[6]  << 16) |
                                 ((int32_t)param->write.value[7]  << 24);
                    int32_t b =   (int32_t)param->write.value[8] |
                                 ((int32_t)param->write.value[9]  <<  8) |
                                 ((int32_t)param->write.value[10] << 16) |
                                 ((int32_t)param->write.value[11] << 24);
                    s_move_cb(x, c, b);
                    ESP_LOGI(TAG, "Move: X=%d, C=%d, B=%d", x, c, b);
                } else {
                    ESP_LOGW(TAG, "Move write length %d (expected 12)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Microsteps write
            else if (handle == handle_table[IDX_CHAR_VAL_MICROSTEPS]) {
                if (param->write.len == 3 && s_microsteps_cb) {
                    memcpy(microsteps_val, param->write.value, 3);
                    ESP_LOGI(TAG, "Microsteps: X=%d, C=%d, B=%d", microsteps_val[0], microsteps_val[1], microsteps_val[2]);
                    s_microsteps_cb(param->write.value[0], param->write.value[1], param->write.value[2]);
                } else {
                    ESP_LOGW(TAG, "Microsteps write length %d (expected 3)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
             // Handle CFG Run current write
            else if (handle == handle_table[IDX_CHAR_VAL_RUN_CURRENT]) {
                if (param->write.len == 6 && s_run_current_cb) {
                    memcpy(run_current_val, param->write.value, 6);
                    uint16_t x = run_current_val[0] | (run_current_val[1] << 8);
                    uint16_t c = run_current_val[2] | (run_current_val[3] << 8);
                    uint16_t b = run_current_val[4] | (run_current_val[5] << 8);
                    ESP_LOGI(TAG, "Run current: X=%d mA, C=%d mA, B=%d mA", x, c, b);
                    s_run_current_cb(x, c, b);
                } else {
                    ESP_LOGW(TAG, "Run current write length %d (expected 6)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }

            // Handle CFG Hold current write
            else if (handle == handle_table[IDX_CHAR_VAL_HOLD_CURRENT]) {
                if (param->write.len == 6 && s_hold_current_cb) {
                    memcpy(hold_current_val, param->write.value, 6);
                    uint16_t x = hold_current_val[0] | (hold_current_val[1] << 8);
                    uint16_t c = hold_current_val[2] | (hold_current_val[3] << 8);
                    uint16_t b = hold_current_val[4] | (hold_current_val[5] << 8);
                    ESP_LOGI(TAG, "Hold current: X=%d mA, C=%d mA, B=%d mA", x, c, b);
                    s_hold_current_cb(x, c, b);
                } else {
                    ESP_LOGW(TAG, "Hold current write length %d (expected 6)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Axis unit write
            else if (handle == handle_table[IDX_CHAR_VAL_AXIS_UNIT]) {
                if (param->write.len == 1 && s_axis_unit_cb) {
                    memcpy(axis_unit_val, param->write.value, 1);
                    bool x_deg = axis_unit_val[0] & 0x01;
                    bool c_deg = axis_unit_val[0] & 0x02;
                    bool b_deg = axis_unit_val[0] & 0x04;
                    s_axis_unit_cb(x_deg, c_deg, b_deg);
                    ESP_LOGI(TAG, "Axis unit: X=%s, C=%s, B=%s",
                            x_deg ? "deg" : "mm",
                            c_deg ? "deg" : "mm",
                            b_deg ? "deg" : "mm");
                }
                
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Units per step write
            else if (handle == handle_table[IDX_CHAR_VAL_UNITS_PER_STEP]) {
                if (param->write.len == 12) {
                    if (s_units_per_step_cb) {
                        float x, c, b;
                        memcpy(&x, param->write.value, 4);
                        memcpy(&c, param->write.value + 4, 4);
                        memcpy(&b, param->write.value + 8, 4);
                        s_units_per_step_cb(x, c, b);
                        ESP_LOGI(TAG, "Units per step: X=%f, C=%f, B=%f", x, c, b);
                    } else {
                        ESP_LOGW(TAG, "No callback for Units per step");
                    }
                } else {
                    ESP_LOGW(TAG, "Units per step write length %d (expected 12)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Axis speed write
            else if (handle == handle_table[IDX_CHAR_VAL_AXIS_SPEED]) {
                if (param->write.len == 6) {
                    if (s_axis_speed_cb) {
                        uint16_t x = param->write.value[0] | (param->write.value[1] << 8);
                        uint16_t c = param->write.value[2] | (param->write.value[3] << 8);
                        uint16_t b = param->write.value[4] | (param->write.value[5] << 8);
                        s_axis_speed_cb(x, c, b);
                        ESP_LOGI(TAG, "Axis speed: X=%u, C=%u, B=%u", x, c, b);
                    } else {
                        ESP_LOGW(TAG, "No callback for Axis speed");
                    }
                } else {
                    ESP_LOGW(TAG, "Axis speed write length %d (expected 6)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Axis acceleration write
            else if (handle == handle_table[IDX_CHAR_VAL_AXIS_ACCEL]) {
                if (param->write.len == 6) {
                    if (s_axis_accel_cb) {
                        uint16_t x = param->write.value[0] | (param->write.value[1] << 8);
                        uint16_t c = param->write.value[2] | (param->write.value[3] << 8);
                        uint16_t b = param->write.value[4] | (param->write.value[5] << 8);
                        s_axis_accel_cb(x, c, b);
                        ESP_LOGI(TAG, "Axis acceleration: X=%u, C=%u, B=%u", x, c, b);
                    } else {
                        ESP_LOGW(TAG, "No callback for Axis acceleration");
                    }
                } else {
                    ESP_LOGW(TAG, "Axis acceleration write length %d (expected 6)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Virtual limit write
            else if (handle == handle_table[IDX_CHAR_VAL_VIRTUAL_LIMIT]) {
                if (param->write.len == 1) {
                    if (s_virtual_limit_cb) {
                        uint8_t val = param->write.value[0];
                        bool x_en = (val & 0x01) != 0;
                        bool c_en = (val & 0x02) != 0;
                        bool b_en = (val & 0x04) != 0;
                        s_virtual_limit_cb(x_en, c_en, b_en);
                        ESP_LOGI(TAG, "Virtual limit: X=%d, C=%d, B=%d", x_en, c_en, b_en);
                    } else {
                        ESP_LOGW(TAG, "No callback for Virtual limit");
                    }
                } else {
                    ESP_LOGW(TAG, "Virtual limit write length %d (expected 1)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Stealthchop write
            else if (handle == handle_table[IDX_CHAR_VAL_STEALTHCHOP]) {
                if (param->write.len == 1) {
                    if (s_stealthchop_cb) {
                        uint8_t val = param->write.value[0];
                        bool x_en = (val & 0x01) != 0;
                        bool c_en = (val & 0x02) != 0;
                        bool b_en = (val & 0x04) != 0;
                        memcpy(stealthchop_val, param->write.value, 1);
                        s_stealthchop_cb(x_en, c_en, b_en);
                        ESP_LOGI(TAG, "Stealthchop: X=%d, C=%d, B=%d", x_en, c_en, b_en);
                    } else {
                        ESP_LOGW(TAG, "No callback for Stealthchop");
                    }
                } else {
                    ESP_LOGW(TAG, "Stealthchop write length %d (expected 1)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CFG Invert direction write
            else if (handle == handle_table[IDX_CHAR_VAL_INVERT_DIR]) {
                if (param->write.len == 1) {
                    if (s_invert_dir_cb) {
                        uint8_t val = param->write.value[0];
                        bool x_inv = (val & 0x01) != 0;
                        bool c_inv = (val & 0x02) != 0;
                        bool b_inv = (val & 0x04) != 0;
                        memcpy(invert_dir_val, param->write.value, 1);
                        s_invert_dir_cb(x_inv, c_inv, b_inv);
                        ESP_LOGI(TAG, "Invert direction: X=%d, C=%d, B=%d", x_inv, c_inv, b_inv);
                    } else {
                        ESP_LOGW(TAG, "No callback for Invert direction");
                    }
                } else {
                    ESP_LOGW(TAG, "Invert direction write length %d (expected 1)", param->write.len);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle OTA control write
            else if (handle == handle_table[IDX_CHAR_VAL_OTA_CONTROL] && s_ota_control_cb) {
                s_ota_control_cb(param->write.value, param->write.len);
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle OTA data write
            else if (handle == handle_table[IDX_CHAR_VAL_OTA_DATA] && s_ota_data_cb) {
                if (param->write.is_prep) {
                    // Prepared (long) write part: clients whose chunk size
                    // exceeds MTU-3 arrive here. Buffer the part and echo it
                    // back in the response as the ATT spec requires; the data
                    // is flashed on ESP_GATTS_EXEC_WRITE_EVT.
                    esp_gatt_status_t status = ESP_GATT_OK;
                    if (param->write.offset > sizeof(ota_prep_buf)) {
                        status = ESP_GATT_INVALID_OFFSET;
                    } else if (param->write.offset + param->write.len > sizeof(ota_prep_buf)) {
                        status = ESP_GATT_INVALID_ATTR_LEN;
                    } else {
                        memcpy(ota_prep_buf + param->write.offset, param->write.value, param->write.len);
                        uint16_t end = param->write.offset + param->write.len;
                        if (end > ota_prep_len) ota_prep_len = end;
                    }
                    if (param->write.need_rsp) {
                        esp_gatt_rsp_t rsp = {0};
                        rsp.attr_value.handle = handle;
                        rsp.attr_value.offset = param->write.offset;
                        rsp.attr_value.len = param->write.len;
                        rsp.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                        memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, &rsp);
                    }
                } else {
                    // Plain write request: the flash write completes BEFORE
                    // the response is sent (RSP_BY_APP) — per-chunk flow
                    // control for the OTA transfer.
                    s_ota_data_cb(param->write.value, param->write.len);
                    if (param->write.need_rsp) {
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }
                }
            }
            // Handle CCCD for LIMIT
            else if (param->write.handle == handle_table[IDX_CHAR_LIMIT_CFG]) {
                if (param->write.len == 2) {
                    limit_notify_enabled = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "LIMIT notifications %s", limit_notify_enabled ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CCCD for HOME
            else if (param->write.handle == handle_table[IDX_CHAR_HOME_CFG]) {
                if (param->write.len == 2) {
                    home_notify_enabled = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "HOME notifications %s", home_notify_enabled ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CCCD for BATT_LEVEL
            else if (param->write.handle == handle_table[IDX_CHAR_BATT_LEVEL_CFG]) {
                if (param->write.len == 2) {
                    batt_level_notify = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "Battery level notifications %s", batt_level_notify ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CCCD for PWR_INFO
            else if (param->write.handle == handle_table[IDX_CHAR_PWR_INFO_CFG]) {
                if (param->write.len == 2) {
                    pwr_info_notify = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "Power info notifications %s", pwr_info_notify ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            // Handle CCCD for PWR_INFO_STR
            else if (param->write.handle == handle_table[IDX_CHAR_PWR_INFO_STR_CFG]) {
                if (param->write.len == 2) {
                    pwr_info_str_notify = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "Power info string notifications %s", pwr_info_str_notify ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
             // Handle CCCD for VERSION
            else if (param->write.handle == handle_table[IDX_CHAR_VERSION_CFG]) {
                if (param->write.len == 2) {
                    version_notify_enabled = (param->write.value[0] == 0x01);
                    ESP_LOGI(TAG, "Version notifications %s", version_notify_enabled ? "enabled" : "disabled");
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            break;
        }

        case ESP_GATTS_EXEC_WRITE_EVT:
        {
            // Finish (or cancel) a prepared long write to OTA_DATA
            if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC &&
                ota_prep_len > 0 && s_ota_data_cb) {
                s_ota_data_cb(ota_prep_buf, ota_prep_len);
            }
            ota_prep_len = 0;
            esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id, ESP_GATT_OK, NULL);
            break;
        }

        case ESP_GATTS_READ_EVT:
        {
            uint16_t handle = param->read.handle;
            if (handle == handle_table[IDX_CHAR_VAL_LIMIT] && s_limit_read_cb) {
                ESP_LOGI(TAG, "LIMIT read, handle = %d", param->read.handle);
                bool x = false, c = false, b = false;
                s_limit_read_cb(&x, &c, &b);
                uint8_t limit_mask = (x ? 1 : 0) | (c ? 2 : 0) | (b ? 4 : 0);
                esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_LIMIT], 1, &limit_mask);
            }
            else if (handle == handle_table[IDX_CHAR_VAL_VERSION] && s_version_read_cb) {
                const char *ver = s_version_read_cb();
                esp_ble_gatts_set_attr_value(handle, strlen(ver), (uint8_t*)ver);
            }
            break;
        }

        default:
            break;
    }
}

// ----------------------------- GAP Event Handler -----------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Adv data set complete");
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: %s", esp_err_to_name(param->adv_start_cmpl.status));
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
                if (s_start_cb) s_start_cb();
            }
            break;
        default:
            break;
    }
}

// ----------------------------- Start Advertising -----------------------------------
static void start_advertising(void)
{
    // Use simple advertising parameters
    esp_ble_adv_params_t adv_params = {
        .adv_int_min        = 0x20,   // 32 ms
        .adv_int_max        = 0x40,   // 64 ms
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    esp_ble_gap_config_adv_data(&adv_data);
    // Wait for ADV_DATA_SET_COMPLETE, then start advertising
    // For simplicity, we start advertising immediately; it's okay.
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
    }
}

// ----------------------------- Public API ------------------------------------------
void ble_init(
    const char *device_name,
    ble_start_cb_t on_start_cb,
    ble_connect_cb_t on_connect_cb,
    ble_disconnect_cb_t on_disconnect_cb,
    ble_version_read_cb_t version_read_cb,
    ble_ota_control_cb_t ota_control_cb,
    ble_ota_data_cb_t ota_data_cb,
    ble_mot_en_cb_t mot_en_cb,
    ble_home_cmd_cb_t home_cmd_cb,
    ble_move_cb_t move_cb,
    ble_limit_read_cb_t limit_read_cb,
    ble_microsteps_cb_t microsteps_cb,
    ble_run_current_cb_t run_current_cb,
    ble_hold_current_cb_t hold_current_cb,
    ble_axis_unit_cb_t axis_unit_cb,
    ble_units_per_step_cb_t units_per_step_cb,
    ble_axis_speed_cb_t axis_speed_cb,
    ble_axis_accel_cb_t axis_accel_cb,
    ble_virtual_limit_cb_t virtual_limit_cb,
    ble_stealthchop_cb_t stealthchop_cb,
    ble_invert_dir_cb_t invert_dir_cb)
{
    s_start_cb = on_start_cb;
    s_connect_cb = on_connect_cb;
    s_disconnect_cb = on_disconnect_cb;

    s_version_read_cb = version_read_cb;
    s_ota_control_cb = ota_control_cb;
    s_ota_data_cb = ota_data_cb;

    s_mot_en_cb = mot_en_cb;
    s_home_cmd_cb = home_cmd_cb;
    s_move_cb = move_cb;
    s_limit_read_cb = limit_read_cb;
    
    s_microsteps_cb = microsteps_cb;
    s_run_current_cb = run_current_cb;
    s_hold_current_cb = hold_current_cb;
    s_axis_unit_cb = axis_unit_cb;
    s_units_per_step_cb = units_per_step_cb;
    s_axis_speed_cb = axis_speed_cb;
    s_axis_accel_cb = axis_accel_cb;
    s_virtual_limit_cb = virtual_limit_cb;
    s_stealthchop_cb = stealthchop_cb;
    s_invert_dir_cb = invert_dir_cb;

    // Initialize NVS (must be done by application before calling ble_init)
    // Note: we assume app already called nvs_flash_init(); otherwise we do it here.
    // But to avoid coupling, we'll rely on the app to init NVS.

    // Release Classic BT memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // If controller was already initialized, deinit it first
    esp_err_t err = esp_bt_controller_deinit();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "BT controller deinit failed: %s", esp_err_to_name(err));
    }

    // Init BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Init Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Set device name
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(device_name));

    // Register GAP and GATTS callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // Register GATT application (app_id = 0)
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    // Optional: set local MTU
    esp_ble_gatt_set_local_mtu(500);

    ESP_LOGI(TAG, "BLE initialized with name '%s'", device_name);
}

void ble_set_microsteps_value(uint8_t x, uint8_t c, uint8_t b)
{
    microsteps_val[0] = x;
    microsteps_val[1] = c;
    microsteps_val[2] = b;
}

void ble_set_run_current_value(uint16_t x, uint16_t c, uint16_t b)
{
    run_current_val[0] = x & 0xFF;
    run_current_val[1] = (x >> 8) & 0xFF;
    run_current_val[2] = c & 0xFF;
    run_current_val[3] = (c >> 8) & 0xFF;
    run_current_val[4] = b & 0xFF;
    run_current_val[5] = (b >> 8) & 0xFF;
}

void ble_set_hold_current_value(uint16_t x, uint16_t c, uint16_t b)
{
    hold_current_val[0] = x & 0xFF;
    hold_current_val[1] = (x >> 8) & 0xFF;
    hold_current_val[2] = c & 0xFF;
    hold_current_val[3] = (c >> 8) & 0xFF;
    hold_current_val[4] = b & 0xFF;
    hold_current_val[5] = (b >> 8) & 0xFF;
}

void ble_set_axis_unit_value(bool x_deg, bool c_deg, bool b_deg)
{
    uint8_t packed = 0;
    if (x_deg) packed |= 0x01;
    if (c_deg) packed |= 0x02;
    if (b_deg) packed |= 0x04;
    axis_unit_val[0] = packed;
}

void ble_set_units_per_step_value(float x, float c, float b)
{
    memcpy(&units_per_step_val[0], &x, 4);
    memcpy(&units_per_step_val[4], &c, 4);
    memcpy(&units_per_step_val[8], &b, 4);
}

void ble_set_axis_speed_value(uint16_t x, uint16_t c, uint16_t b)
{
    axis_speed_val[0] = x & 0xFF;
    axis_speed_val[1] = (x >> 8) & 0xFF;
    axis_speed_val[2] = c & 0xFF;
    axis_speed_val[3] = (c >> 8) & 0xFF;
    axis_speed_val[4] = b & 0xFF;
    axis_speed_val[5] = (b >> 8) & 0xFF;
}

void ble_set_axis_accel_value(uint16_t x, uint16_t c, uint16_t b)
{
    axis_accel_val[0] = x & 0xFF;
    axis_accel_val[1] = (x >> 8) & 0xFF;
    axis_accel_val[2] = c & 0xFF;
    axis_accel_val[3] = (c >> 8) & 0xFF;
    axis_accel_val[4] = b & 0xFF;
    axis_accel_val[5] = (b >> 8) & 0xFF;
}

void ble_set_virtual_limit_value(bool x_en, bool c_en, bool b_en)
{
    virtual_limit_val[0] = (x_en ? 0x01 : 0) | (c_en ? 0x02 : 0) | (b_en ? 0x04 : 0);
}

// ----------------------------- Send Limit Notification -----------------------------
// The limit state is published only when a switch changes, so — as with the battery level —
// the value is stored even while no client is connected, otherwise a client connecting after
// a change would read a stale mask.
void ble_set_limit(
    bool x_limited,
    bool c_limited,
    bool b_limited)
{
    if (!attr_table_ready)
        return;

    uint8_t limit_mask = (x_limited ? 1 : 0) | (c_limited ? 2 : 0) | (b_limited ? 4 : 0);

    // Update characteristic value first (optional, but keep local copy)
    esp_ble_gatts_set_attr_value(
        handle_table[IDX_CHAR_VAL_LIMIT], sizeof(limit_mask), &limit_mask);

    // Send notification (indicate = false)
    if (connected && limit_notify_enabled) {
        esp_ble_gatts_send_indicate(
            s_gatts_if,
            conn_id,
            handle_table[IDX_CHAR_VAL_LIMIT],
            sizeof(limit_mask),
            &limit_mask,
            false);
    }
}

// Update home status with notification
void ble_set_home_status(
    bool x_requested,
    bool c_requested,
    bool b_requested,
    bool x_homed,
    bool c_homed,
    bool b_homed)
{
    if (!connected)
        return;

    uint8_t status = 0;

    // High nibble: requested bits (bits 4-6)
    if (x_requested) status |= 0x10;
    if (c_requested) status |= 0x20;
    if (b_requested) status |= 0x40;

    // Low nibble: homed bits (bits 0-2)
    if (x_homed) status |= 0x01;
    if (c_homed) status |= 0x02;
    if (b_homed) status |= 0x04;

    esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_HOME], sizeof(status), &status);

    if (home_notify_enabled) {
        esp_ble_gatts_send_indicate(
            s_gatts_if,
            conn_id,
            handle_table[IDX_CHAR_VAL_HOME],
            sizeof(status),
            &status,
            false);
    }
}

// Set MOT_EN characteristic value (no notify)
void ble_set_mot_en_state(uint8_t enable)
{
    esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_MOT_EN], 1, &enable);
}

// Set BATT_LEVEL characteristic value with notification.
// The value is stored even while no client is connected: the battery level is
// published only when it changes, so a client connecting later must be able to
// read the last known level instead of the attribute's initial zero.
void ble_set_battery_level(uint8_t percent) {
    if (!attr_table_ready)
        return;

    if (percent > 100)
        percent = 100;

    uint8_t level = (uint8_t)(percent * 255 / 100);  // 0-100 → 0-255

    esp_ble_gatts_set_attr_value(
        handle_table[IDX_CHAR_VAL_BATT_LEVEL],
        1,
        &level);

    if (connected && batt_level_notify) {
        esp_ble_gatts_send_indicate(
            s_gatts_if,
            conn_id,
            handle_table[IDX_CHAR_VAL_BATT_LEVEL],
            1,
            &level,
            false);
    }
}

// Set PWR_INFO characteristic value with notification.
// Stored even while disconnected, for the same reason as the battery level: the INA219
// reports only when its readings move, so a client connecting to a device on a steady
// supply would otherwise read zeroes.
void ble_set_power_info(float voltage, float current, float power) {
    if (!attr_table_ready)
        return;

    uint8_t buf[12];
    memcpy(buf, &voltage, 4);
    memcpy(buf+4, &current, 4);
    memcpy(buf+8, &power, 4);
    
    esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_PWR_INFO], sizeof(buf), buf);
    
    if (connected && pwr_info_notify) {
        esp_ble_gatts_send_indicate(
            s_gatts_if,
            conn_id,
            handle_table[IDX_CHAR_VAL_PWR_INFO],
            sizeof(buf),
            buf,
            false);
    }
}

// Set PWR_INFO_STR characteristic value with notification (stored while disconnected too,
// see ble_set_power_info)
void ble_set_power_info_string(const char* str) {
    if (!attr_table_ready)
        return;

    const int max_len = 40;
    size_t len = strlen(str);
    if (len > max_len)
        len = max_len;

    esp_ble_gatts_set_attr_value(
        handle_table[IDX_CHAR_VAL_PWR_INFO_STR], 
        len,
        (uint8_t*)str);

    if (connected && pwr_info_str_notify) {
        esp_ble_gatts_send_indicate(
            s_gatts_if,
            conn_id,
            handle_table[IDX_CHAR_VAL_PWR_INFO_STR],
            len,
            (uint8_t*)str,
            false);
    }
}

// Update BLE_VERSION characteristic value with notification
void ble_update_firmware_version(void)
{
    if (!connected) return;
    if (!s_version_read_cb) return;
    const char *ver = s_version_read_cb();
    esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_VERSION], strlen(ver), (uint8_t*)ver);
    if (version_notify_enabled) {
        esp_ble_gatts_send_indicate(s_gatts_if, conn_id, handle_table[IDX_CHAR_VAL_VERSION],
                                    strlen(ver), (uint8_t*)ver, false);
    }
}