#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "led_strip.h"
#include "i2c_manager.h"
#include "pca9555.h"
#include "ina219.h"
#include "nvs_flash.h"
#include "ble.h"
#include "ota.h"
#include "motion.h"
#include "app_config.h"
#include "nvs_config.h"

#define TMC2130_BUS_NUM_DRIVERS 3

#include "tmc2130.h"
#include "Arduino.h"
#include "FastAccelStepper.h"

#define SPI_BUS_HOST        SPI2_HOST
#define SPI_BUS_CLK_HZ      1000000
#define SPI_BUS_CLK_IO      GPIO_NUM_14
#define SPI_BUS_MOSI_IO     GPIO_NUM_13
#define SPI_BUS_MISO_IO     GPIO_NUM_12
#define STEPPERS_EN_IO      GPIO_NUM_15

#define AXIS_X_ID           0
#define AXIS_X_SPI_CS_IO    GPIO_NUM_21
#define AXIS_X_DIR_IO       GPIO_NUM_23
#define AXIS_X_STEP_IO      GPIO_NUM_22

#define AXIS_C_ID           1
#define AXIS_C_SPI_CS_IO    GPIO_NUM_17
#define AXIS_C_DIR_IO       GPIO_NUM_18
#define AXIS_C_STEP_IO      GPIO_NUM_19

#define AXIS_B_ID           2
#define AXIS_B_SPI_CS_IO    GPIO_NUM_25
#define AXIS_B_DIR_IO       GPIO_NUM_26
#define AXIS_B_STEP_IO      GPIO_NUM_27

#define FAN_PWM_IO          GPIO_NUM_5
#define FAN_PWM_FREQ        1000
#define FAN_PWM_RES         LEDC_TIMER_8_BIT
#define FAN_PWM_DUTY        76

#define RGB_LED_IO          GPIO_NUM_16
#define RGB_LED_NUM         1

#define I2C_SCL_IO          GPIO_NUM_33
#define I2C_SDA_IO          GPIO_NUM_32

#define INA219_ADDR         0x40
#define INA219_FREQ_HZ      100000

#define PCA9555_ADDR        0x20
#define PCA9555_FREQ_HZ     100000
#define PCA9555_INT_IO      GPIO_NUM_35

#define PCA9555_TMC_B_DIAG0         0
#define PCA9555_TMC_B_DIAG1         1
#define PCA9555_ENDSTOP_B           2
#define PCA9555_TMC_C_DIAG0         3
#define PCA9555_TMC_C_DIAG1         4
#define PCA9555_ENDSTOP_C           5
#define PCA9555_TMC_X_DIAG0         6
#define PCA9555_TMC_X_DIAG1         7
#define PCA9555_ENDSTOP_X           8

static const char *TAG = "AGO_SLIDER";

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper1 = NULL;
FastAccelStepper* stepper2 = NULL;
FastAccelStepper* stepper3 = NULL;

static led_strip_handle_t led_strip = NULL;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t ina219_dev = NULL;
static i2c_master_dev_handle_t pca9555_dev = NULL;

static PCA9555 pca9555;

static bool ble_is_advertising = false;
static bool ble_is_connected = false;

static uint8_t last_batt_lvl_percent = 255;  // 255 = no INA219 reading yet

void init_tmc2130(const app_config_t *cfg);
void init_fastAccelStepper(const app_config_t *cfg);

void init_fan_pwm();
void init_led();
void set_led_color(uint8_t r, uint8_t g, uint8_t b);
void init_i2c_manager();
void i2c_scan();
void init_ina219();
void init_pca9555();

static void on_ble_start(void);
static void on_ble_connect(void);
static void on_ble_disconnect(void);
static void led_indicator_task(void *arg);

static void on_ble_move(int32_t x_speed, int32_t y_speed, int32_t z_speed);
static void on_ble_mot_en(bool enable);
static void on_ble_home(bool home_x, bool home_c, bool home_b);
static void on_ble_limit_read(bool *x, bool *c, bool *b);

static void on_limit_change(uint8_t pin, bool value);

static void on_home_progress(bool x_req, bool c_req, bool b_req, bool x_homed, bool c_homed, bool b_homed);

static void on_ina219_data(float voltage, float current, float power);

static void on_microsteps(uint8_t x, uint8_t c, uint8_t b);
static void on_run_current(uint16_t x, uint16_t c, uint16_t b);
static void on_hold_current(uint16_t x, uint16_t c, uint16_t b);
static void on_axis_unit(bool x_deg, bool c_deg, bool b_deg);
static void on_units_per_step(float x, float c, float b);
static void on_axis_speed(uint16_t x, uint16_t c, uint16_t b);
static void on_axis_accel(uint16_t x, uint16_t c, uint16_t b);
static void on_virtual_limit(bool x_en, bool c_en, bool b_en);
static void on_stealthchop(bool x_en, bool c_en, bool b_en);
static void on_invert_dir(bool x_inv, bool c_inv, bool b_inv);

static const char* on_version_read(void);
static void on_ota_control(const uint8_t* data, size_t len);
static void on_ota_data(const uint8_t* data, size_t len);

static void ota_progress(size_t received, size_t total);
static void ota_status(const char* msg, bool err);
static void ota_complete(bool success, const char* msg);

static void ble_apply_config(const app_config_t *cfg);
static void motion_apply_config(const app_config_t *cfg);

extern "C" void app_main(void)
{
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);

    // Initialize NVS (required by BLE)
    //nvs_flash_erase(); // <-- SHOULD BE commented. UNCOMMENT TO RESET CONFIG IN NVS FLASH.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(nvs_config_init());

    const app_config_t *cfg = nvs_config_get();
    if (cfg) {
        ble_apply_config(cfg);
        motion_apply_config(cfg);
    }

    init_led();
    set_led_color(12, 1, 1);
    xTaskCreate(led_indicator_task, "led_indicator", 2048, NULL, 1, NULL);

    init_tmc2130(cfg);
    init_fastAccelStepper(cfg);
    
    if (!stepper1 || !stepper2 || !stepper3) {
        ESP_LOGE(TAG, "One or more steppers not initialized!");
        return;
    }

    // init_fan_pwm();   

    init_i2c_manager();
    // i2c_scan();
    init_ina219();
    init_pca9555();

    motion_init(
        stepper1, stepper2, stepper3,
        &pca9555,
        PCA9555_ENDSTOP_X,
        PCA9555_ENDSTOP_C,
        PCA9555_ENDSTOP_B,
        on_home_progress);

    // Initialize BLE (everything inside)
    ble_init(
        "AGO Slider",
        on_ble_start,
        on_ble_connect,
        on_ble_disconnect,
        on_version_read,
        on_ota_control,
        on_ota_data,
        on_ble_mot_en,
        on_ble_home,
        on_ble_move,
        on_ble_limit_read,
        on_microsteps,
        on_run_current,
        on_hold_current,
        on_axis_unit,
        on_units_per_step,
        on_axis_speed,
        on_axis_accel,
        on_virtual_limit,
        on_stealthchop,
        on_invert_dir);

    // All critical subsystems are up: confirm the running image so the
    // bootloader does not roll back to the previous firmware after an OTA
    // update (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is set).
    ota_confirm_running_image();
}

// -----------------------------------------------------------------------------
// WS2812B LED
// -----------------------------------------------------------------------------

void init_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_IO,
        .max_leds = RGB_LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,       // 10 MHz
        .mem_block_symbols = 64,                 // RMT memory size
        .flags = {
            .with_dma = false,                   // no need DMA for the only led
        }
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    led_strip_clear(led_strip);
    
    ESP_LOGI("LED", "WS2812B initialized on GPIO%d", RGB_LED_IO);
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

static void led_indicator_task(void *arg) {
    while (1) {
        if (ble_is_connected) {
            set_led_color(0, 0, 5);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else if (ble_is_advertising) {
            set_led_color(0, 0, 5);
            vTaskDelay(pdMS_TO_TICKS(500));
            set_led_color(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else {
            set_led_color(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// -----------------------------------------------------------------------------
// FAN
// -----------------------------------------------------------------------------

void init_fan_pwm()
{
    ledcAttach(FAN_PWM_IO, FAN_PWM_FREQ, FAN_PWM_RES);
    ledcWrite(FAN_PWM_IO, FAN_PWM_DUTY);
}

// -----------------------------------------------------------------------------
// TMC2130
// -----------------------------------------------------------------------------

/**
 * @brief Initialize TMC2130 SPI bus and drivers.
 */
void init_tmc2130(const app_config_t *cfg) {
    ESP_LOGI(TAG, "Initializing TMC2130 bus & drivers...");

    // Extract TMC2130 configuration parameters from config
    uint8_t microsteps_x, microsteps_c, microsteps_b;
    app_config_unpack_microsteps(cfg, &microsteps_x, &microsteps_c, &microsteps_b);
    uint16_t run_current_x, run_current_c, run_current_b,
             hold_current_x, hold_current_c, hold_current_b;
    app_config_unpack_run_current(cfg, &run_current_x, &run_current_c, &run_current_b);
    app_config_unpack_hold_current(cfg, &hold_current_x, &hold_current_c, &hold_current_b);
    bool stealthchop_x, stealthchop_c, stealthchop_b,
         invert_dir_x, invert_dir_c, invert_dir_b;
    app_config_unpack_stealthchop(cfg, &stealthchop_x, &stealthchop_c, &stealthchop_b);
    app_config_unpack_invert_dir(cfg, &invert_dir_x, &invert_dir_c, &invert_dir_b);

    tmc2130_config_t tmc_cfg = {
        .spi_host       = SPI_BUS_HOST,
        .spi_clock_hz   = SPI_BUS_CLK_HZ,
        .sclk_pin       = SPI_BUS_CLK_IO,
        .mosi_pin       = SPI_BUS_MOSI_IO,
        .miso_pin       = SPI_BUS_MISO_IO,
        .en_pin         = STEPPERS_EN_IO,
        .drivers = {
            {
                .cs_pin          = AXIS_X_SPI_CS_IO,
                .step_pin        = AXIS_X_STEP_IO,
                .dir_pin         = AXIS_X_DIR_IO,
                .microsteps      = microsteps_x,
                .run_current_ma  = run_current_x,
                .hold_current_ma = hold_current_x,
                .stealthchop     = stealthchop_x,
                .invert_dir      = invert_dir_x,
            },
            {
                .cs_pin          = AXIS_C_SPI_CS_IO,
                .step_pin        = AXIS_C_STEP_IO,
                .dir_pin         = AXIS_C_DIR_IO,
                .microsteps      = microsteps_c,
                .run_current_ma  = run_current_c,
                .hold_current_ma = hold_current_c,
                .stealthchop     = stealthchop_c,
                .invert_dir      = invert_dir_c,
            },
            {
                .cs_pin          = AXIS_B_SPI_CS_IO,
                .step_pin        = AXIS_B_STEP_IO,
                .dir_pin         = AXIS_B_DIR_IO,
                .microsteps      = microsteps_b,
                .run_current_ma  = run_current_b,
                .hold_current_ma = hold_current_b,
                .stealthchop     = stealthchop_b,
                .invert_dir      = invert_dir_b,
            }
        }
    };
    
    esp_err_t res = tmc2130_init(&tmc_cfg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "TMC2130 bus & drivers initialization failed: %s", esp_err_to_name(res));
        return;
    } else {
        ESP_LOGI(TAG, "TMC2130 bus & drivers initialization succeeded!");
    }
}

// -----------------------------------------------------------------------------
// FastAccelStepper
// -----------------------------------------------------------------------------

/**
 * @brief Initialize FastAccelStepper for STEP/DIR pulse generation.
 */
void init_fastAccelStepper(const app_config_t *cfg)
{
    // Extract required configuration parameters from config
    uint16_t speed_x, speed_c, speed_b; // step/second
    uint16_t accel_x, accel_c, accel_b; // step/second²

    app_config_unpack_axis_speed(cfg, &speed_x, &speed_c, &speed_b);
    app_config_unpack_axis_accel(cfg, &accel_x, &accel_c, &accel_b);

    engine.init(0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    stepper1 = engine.stepperConnectToPin(AXIS_X_STEP_IO);
    if (stepper1) {
        stepper1->setDirectionPin(AXIS_X_DIR_IO);
        stepper1->setEnablePin(STEPPERS_EN_IO); // Shared EN
        stepper1->setAutoEnable(false);         // Disable autoenable

        stepper1->setSpeedInUs(((speed_x > 0) ? (10000000 / speed_x) : 10000));  // the parameter is us/step !!!
        stepper1->setAcceleration(accel_x);

        ESP_LOGI(TAG, "FastAccelStepper stepper #%d initialized!", AXIS_X_ID);
    } else {
        ESP_LOGE(TAG, "FastAccelStepper stepper #%d was not initialized!", AXIS_X_ID);
    }

    stepper2 = engine.stepperConnectToPin(AXIS_C_STEP_IO);
    if (stepper2) {
        stepper2->setDirectionPin(AXIS_C_DIR_IO);
        stepper2->setEnablePin(STEPPERS_EN_IO); // Shared EN
        stepper2->setAutoEnable(false);         // Disable autoenable
        
        stepper2->setSpeedInUs(((speed_c > 0) ? (10000000 / speed_c) : 10000));
        stepper2->setAcceleration(accel_c);

        ESP_LOGI(TAG, "FastAccelStepper stepper #%d initialized!", AXIS_C_ID);
    } else {
        ESP_LOGE(TAG, "FastAccelStepper stepper #%d was not initialized!", AXIS_C_ID);
    }

    stepper3 = engine.stepperConnectToPin(AXIS_B_STEP_IO);
    if (stepper3) {
        stepper3->setDirectionPin(AXIS_B_DIR_IO);
        stepper3->setEnablePin(STEPPERS_EN_IO); // Shared EN
        stepper3->setAutoEnable(false);  
        
        stepper3->setSpeedInUs(((speed_b > 0) ? (10000000 / speed_b) : 10000));
        stepper3->setAcceleration(accel_b);

        ESP_LOGI(TAG, "FastAccelStepper stepper #%d initialized!", AXIS_B_ID);
    } else {
        ESP_LOGE(TAG, "FastAccelStepper stepper #%d was not initialized!", AXIS_B_ID);
    }
}

// -----------------------------------------------------------------------------
// Motion
// -----------------------------------------------------------------------------

static void motion_apply_config(const app_config_t *cfg)
{
    if (!cfg) return;

    bool x_deg = (cfg->axis_unit & 0x01) != 0;
    bool c_deg = (cfg->axis_unit & 0x02) != 0;
    bool b_deg = (cfg->axis_unit & 0x04) != 0;
    motion_set_axis_unit(x_deg, c_deg, b_deg);

    float x_step, c_step, b_step;
    memcpy(&x_step, cfg->units_per_step, 4);
    memcpy(&c_step, cfg->units_per_step + 4, 4);
    memcpy(&b_step, cfg->units_per_step + 8, 4);
    motion_set_units_per_step(x_step, c_step, b_step);

    bool x_vl = (cfg->virtual_limit & 0x01) != 0;
    bool c_vl = (cfg->virtual_limit & 0x02) != 0;
    bool b_vl = (cfg->virtual_limit & 0x04) != 0;
    motion_set_virtual_limit(x_vl, c_vl, b_vl);
}


// -----------------------------------------------------------------------------
// I2C Manager
// -----------------------------------------------------------------------------

void init_i2c_manager() {
    esp_err_t err = i2c_manager_init(I2C_NUM_0, I2C_SCL_IO, I2C_SDA_IO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return;
    }

    err = i2c_manager_get_bus(&i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get I2C bus handle.");
        return;
    }
}

void i2c_scan() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_master_dev_handle_t test_dev;
        esp_err_t err = i2c_manager_add_device(addr, 100000, &test_dev);
        
        if (err == ESP_OK && test_dev != NULL) {
            uint8_t test_reg = 0x00;
            uint8_t dummy;
            
            esp_err_t probe_err = i2c_master_transmit_receive(test_dev, &test_reg, 1, &dummy, 1, 100);
            
            if (probe_err == ESP_OK) {
                ESP_LOGI(TAG, "Device found at 0x%02X", addr);
            }
            
            i2c_master_bus_rm_device(test_dev);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Scan complete");
}

// -----------------------------------------------------------------------------
// PCA9555
// -----------------------------------------------------------------------------

void init_pca9555 ()
{
    esp_err_t err = i2c_manager_add_device(PCA9555_ADDR, PCA9555_FREQ_HZ, &pca9555_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add PCA9555 to I2C bus.");
        return;
    }
    
    err = pca9555_init(&pca9555, pca9555_dev, PCA9555_INT_IO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9555 device.");
        return;
    }
    
    // Configure pins 0-8 as inputs (first 9 pins)
    for (int pin = 0; pin < 9; pin++) {
        pca9555_set_gpio_direction(&pca9555, pin, PCA9555_DIR_IN);
    }
    
    // Setup handlers for endstop sensors
    pca9555_set_interrupt_handler(&pca9555, PCA9555_ENDSTOP_X, on_limit_change);
    pca9555_set_interrupt_handler(&pca9555, PCA9555_ENDSTOP_C, on_limit_change);
    pca9555_set_interrupt_handler(&pca9555, PCA9555_ENDSTOP_B, on_limit_change);

    // Initialize values  
    bool x_lim, c_lim, b_lim;
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_X, &x_lim);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_C, &c_lim);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_B, &b_lim);

    // Define temporary interrupt
    pca9555_intr_t intr_handler = [](uint8_t pin, bool value) {
        ESP_LOGI(TAG, "PCA9555 GPIO %d changed to %s.", pin, value ? "HIGH" : "LOW");
    };

    // Set temporary interrupt
    err = pca9555_set_interrupt_handler(&pca9555, 0, intr_handler);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt handler for GPIO 0.");
        return;
    }
}


// ============================================================================
// INA219
// ============================================================================

void init_ina219(void) {
    ESP_ERROR_CHECK(i2c_manager_add_device(INA219_ADDR, INA219_FREQ_HZ, &ina219_dev));
    ESP_ERROR_CHECK(ina219_init(ina219_dev, on_ina219_data));
    ESP_ERROR_CHECK(ina219_start());
}

static void on_ina219_data(float voltage, float current, float power) {
    // Calculate battery percentage (0-100) for a 5-cell Li-ion battery (18.0V discharged, 21.5V fully charged)
    const float min_voltage = 18.0f;
    const float max_voltage = 21.5f;
    uint8_t batt_lvl_percent = 0;
    if (voltage >= max_voltage)
        batt_lvl_percent = 100;
    else if (voltage <= min_voltage)
        batt_lvl_percent = 0;
    else 
        batt_lvl_percent = (uint8_t)((voltage - min_voltage) / (max_voltage - min_voltage) * 100.0f);

    if (batt_lvl_percent != last_batt_lvl_percent)
    {
       last_batt_lvl_percent = batt_lvl_percent;
       ble_set_battery_level(batt_lvl_percent);
    }

    ble_set_power_info(voltage, current, power);

    char str[40];
    snprintf(str, sizeof(str), "%.1fV %.2fA %.1fW", voltage, current, power);
    ble_set_power_info_string(str);
}

// -----------------------------------------------------------------------------
// BLE system callbacks
// -----------------------------------------------------------------------------

static void on_ble_start(void) {
    ble_is_advertising = true;
    ble_is_connected = false;
}

static void on_ble_connect(void) {
    ble_is_connected = true;
    ble_is_advertising = false;

    // Both the INA219 (readings past a threshold) and the level check below report only on
    // change, and the first reading is taken before the BLE stack is up, so on a steady supply
    // nothing would ever reach a client: it would read 0% until the level happens to move.
    // Re-publish the last known level so the fresh client can read it right away.
    if (last_batt_lvl_percent <= 100)
        ble_set_battery_level(last_batt_lvl_percent);
}

static void on_ble_disconnect(void) {
    ble_is_connected = false;
    ble_is_advertising = true;
}

// -----------------------------------------------------------------------------
// BLE application characteristics callbacks
// -----------------------------------------------------------------------------

static void on_ble_mot_en(bool enable) {
    ESP_LOGI("Slider", "Set all motors %s", enable ? "enabled" : "disabled");
    motion_set_enable(enable);
    ble_set_mot_en_state(enable);
}

static void on_ble_home(bool home_x, bool home_c, bool home_b) {
    ESP_LOGI("Slider", "Homing: X=%d, Y=%d, Z=%d", home_x, home_c, home_b);
    
    motion_start_homing(home_x, home_c, home_b);
}

static void on_ble_move(int32_t x, int32_t c, int32_t b) {
    ESP_LOGI("Slider", "Moving: X=%d, Y=%d, Z=%d", x, c, b);
    motion_move_relative(x, c, b);
}

static void on_limit_change(uint8_t pin, bool value) {
    ESP_LOGI("PCA9555", "Endstop on pin %d changed it value to %d", pin, value);

    bool x, c, b;
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_X, &x);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_C, &c);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_B, &b);

    ble_set_limit(!x, !c, !b);
}

static void on_ble_limit_read(bool *x, bool *c, bool *b) {
    bool raw_x, raw_c, raw_b;

    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_X, &raw_x);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_C, &raw_c);
    pca9555_get_gpio_value(&pca9555, PCA9555_ENDSTOP_B, &raw_b);

    *x = !raw_x;
    *c = !raw_c;
    *b = !raw_b;
}

static void on_home_progress(bool x_req, bool c_req, bool b_req, bool x_homed, bool c_homed, bool b_homed)
{
    ble_set_home_status(x_req, c_req, b_req, x_homed, c_homed, b_homed);
}

// -----------------------------------------------------------------------------
// BLE configuration characteristics callbacks
// -----------------------------------------------------------------------------
static void on_microsteps(uint8_t x, uint8_t c, uint8_t b) {
    ESP_LOGI(TAG, "Config: microsteps X=%d C=%d B=%d", x, c, b);
    nvs_config_set_microsteps(x, c, b);
    tmc2130_set_microsteps(AXIS_X_ID, x);
    tmc2130_set_microsteps(AXIS_C_ID, c);
    tmc2130_set_microsteps(AXIS_B_ID, b);
}

static void on_run_current(uint16_t x, uint16_t c, uint16_t b) {
    ESP_LOGI(TAG, "Config: run current X=%u mA, C=%u mA, B=%u mA", x, c, b);
    nvs_config_set_run_current(x, c, b);
    tmc2130_set_run_current(AXIS_X_ID, x);
    tmc2130_set_run_current(AXIS_C_ID, c);
    tmc2130_set_run_current(AXIS_B_ID, b);
}

static void on_hold_current(uint16_t x, uint16_t c, uint16_t b) {
    ESP_LOGI(TAG, "Config: hold current X=%u mA, C=%u mA, B=%u mA", x, c, b);
    nvs_config_set_hold_current(x, c, b);
    tmc2130_set_hold_current(AXIS_X_ID, x);
    tmc2130_set_hold_current(AXIS_C_ID, c);
    tmc2130_set_hold_current(AXIS_B_ID, b);
}

static void on_axis_unit(bool x_deg, bool c_deg, bool b_deg) {
    ESP_LOGI(TAG, "Config: axis unit X=%d C=%d B=%d", x_deg, c_deg, b_deg);
    nvs_config_set_axis_unit(x_deg, c_deg, b_deg);
    motion_set_axis_unit(x_deg, c_deg, b_deg);
}

static void on_units_per_step(float x, float c, float b) {
    ESP_LOGI(TAG, "Config: units per step X=%f C=%f B=%f", x, c, b);
    nvs_config_set_units_per_step(x, c, b);
    motion_set_units_per_step(x, c, b);
}

static void on_axis_speed(uint16_t x, uint16_t c, uint16_t b) {
    ESP_LOGI(TAG, "Config: axis speed X=%u C=%u B=%u", x, c, b);
    nvs_config_set_axis_speed(x, c, b);
    stepper1->setSpeedInUs(((x > 0) ? (10000000 / x) : 10000));
    stepper2->setSpeedInUs(((c > 0) ? (10000000 / c) : 10000));
    stepper3->setSpeedInUs(((b > 0) ? (10000000 / b) : 10000));
}

static void on_axis_accel(uint16_t x, uint16_t c, uint16_t b) {
    ESP_LOGI(TAG, "Config: axis acceleration X=%u C=%u B=%u", x, c, b);
    nvs_config_set_axis_accel(x, c, b);
    stepper1->setAcceleration(x);
    stepper2->setAcceleration(c);
    stepper3->setAcceleration(b);
}

static void on_virtual_limit(bool x_en, bool c_en, bool b_en) {
    ESP_LOGI(TAG, "Config: virtual limit X=%d C=%d B=%d", x_en, c_en, b_en);
    nvs_config_set_virtual_limit(x_en, c_en, b_en);
    motion_set_virtual_limit(x_en, c_en, b_en);
}

static void on_stealthchop(bool x_en, bool c_en, bool b_en) {
    ESP_LOGI(TAG, "Config: stealthchop X=%d, C=%d, B=%d", x_en, c_en, b_en);
    nvs_config_set_stealthchop(x_en, c_en, b_en);
    tmc2130_set_stealthchop(AXIS_X_ID, x_en);
    tmc2130_set_stealthchop(AXIS_C_ID, c_en);
    tmc2130_set_stealthchop(AXIS_B_ID, b_en);
}

static void on_invert_dir(bool x_inv, bool c_inv, bool b_inv) {
    ESP_LOGI(TAG, "Config: invert direction X=%d, C=%d, B=%d", x_inv, c_inv, b_inv);
    nvs_config_set_invert_dir(x_inv, c_inv, b_inv);
    tmc2130_set_invert_dir(AXIS_X_ID, x_inv);
    tmc2130_set_invert_dir(AXIS_C_ID, c_inv);
    tmc2130_set_invert_dir(AXIS_B_ID, b_inv);
}

static void ble_apply_config(const app_config_t *cfg)
{
    if (!cfg) return;

    // Microsteps
    ble_set_microsteps_value(cfg->microsteps[0], cfg->microsteps[1], cfg->microsteps[2]);

     // Run current (3 x uint16 mA)
    uint16_t run_x = cfg->run_current[0] | (cfg->run_current[1] << 8);
    uint16_t run_c = cfg->run_current[2] | (cfg->run_current[3] << 8);
    uint16_t run_b = cfg->run_current[4] | (cfg->run_current[5] << 8);
    ble_set_run_current_value(run_x, run_c, run_b);

    // Hold current  (3 x uint16 mA)
    uint16_t hold_x = cfg->hold_current[0] | (cfg->hold_current[1] << 8);
    uint16_t hold_c = cfg->hold_current[2] | (cfg->hold_current[3] << 8);
    uint16_t hold_b = cfg->hold_current[4] | (cfg->hold_current[5] << 8);
    ble_set_hold_current_value(hold_x, hold_c, hold_b);

    // Axis unit (packed byte -> three bool flags: true = degrees)
    bool x_deg = (cfg->axis_unit & 0x01) != 0;
    bool c_deg = (cfg->axis_unit & 0x02) != 0;
    bool b_deg = (cfg->axis_unit & 0x04) != 0;
    ble_set_axis_unit_value(x_deg, c_deg, b_deg);

    // Units per step (floats)
    float x_step, c_step, b_step;
    memcpy(&x_step, cfg->units_per_step, 4);
    memcpy(&c_step, cfg->units_per_step + 4, 4);
    memcpy(&b_step, cfg->units_per_step + 8, 4);
    ble_set_units_per_step_value(x_step, c_step, b_step);

    // Axis speed (uint16)
    uint16_t x_home = cfg->axis_speed[0] | (cfg->axis_speed[1] << 8);
    uint16_t c_home = cfg->axis_speed[2] | (cfg->axis_speed[3] << 8);
    uint16_t b_home = cfg->axis_speed[4] | (cfg->axis_speed[5] << 8);
    ble_set_axis_speed_value(x_home, c_home, b_home);

    // Axis acceleration (uint16)
    uint16_t x_accel = cfg->axis_accel[0] | (cfg->axis_accel[1] << 8);
    uint16_t c_accel = cfg->axis_accel[2] | (cfg->axis_accel[3] << 8);
    uint16_t b_accel = cfg->axis_accel[4] | (cfg->axis_accel[5] << 8);
    ble_set_axis_accel_value(x_accel, c_accel, b_accel);

    // Virtual limit
    ble_set_virtual_limit_value(
        (cfg->virtual_limit & 0x01) != 0,
        (cfg->virtual_limit & 0x02) != 0,
        (cfg->virtual_limit & 0x04) != 0
    );
}

// -----------------------------------------------------------------------------
// BLE OTA
// -----------------------------------------------------------------------------

static const char* on_version_read(void)
{
    return ota_get_current_version();
}

static void on_ota_control(const uint8_t* data, size_t len)
{
    // parse command as in original Arduino project
    if (len == 0) return;
    switch (data[0]) {
        case 0x01: // start
            if (len >= 5) {
                uint32_t size = data[1] | (data[2]<<8) | (data[3]<<16) | (data[4]<<24);
                if (!ota_begin(size, ota_progress, ota_status, ota_complete)) {
                    ESP_LOGE(TAG, "OTA start failed");
                }
            }
            break;
        case 0x02: // end
            if (!ota_end()) ESP_LOGE(TAG, "OTA end failed");
            break;
        case 0x03: // abort
            ota_abort();
            break;
        default:
            ESP_LOGE(TAG, "Unknown OTA command: 0x%02x", data[0]);
    }
}

static void on_ota_data(const uint8_t* data, size_t len)
{
    ota_write(data, len);
}

static void ota_progress(size_t received, size_t total) {
    ESP_LOGD("OTA", "Progress: %d/%d (%.1f%%)", received, total, (received*100.0)/total);
}

static void ota_status(const char* msg, bool err) {
    if (err) ESP_LOGE("OTA", "%s", msg);
    else ESP_LOGI("OTA", "%s", msg);
}

static void ota_complete(bool success, const char* msg) {
    if (success) ESP_LOGI("OTA", "SUCCESS: %s", msg);
    else ESP_LOGE("OTA", "FAILED: %s", msg);
}
