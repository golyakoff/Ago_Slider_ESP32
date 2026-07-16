FILE(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES 
        tmc2130
        esp_driver_spi
        FastAccelStepper
        arduino-esp32
        i2c_manager
        pca9555
        ina219
        nvs_flash
        ble
        ota
        motion
        app_config
        nvs_config
        log
    PRIV_REQUIRES
        spi_flash
        driver
        esp_timer
        bt
)