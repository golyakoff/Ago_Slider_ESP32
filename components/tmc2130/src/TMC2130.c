#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"

#include "tmc2130_trinamic.h"
#include "tmc2130.h"

static const char *TAG = "TMC2130";

typedef struct {
    bool initialized;
    int cs_pin;
    int step_pin;
    int dir_pin;
    spi_device_handle_t spi_handle;
    uint8_t microsteps;
    uint16_t run_current_ma;
    uint16_t hold_current_ma;
    uint8_t hold_delay_ms;
    bool stealthchop;
    bool invert_dir;
} driver_state_t;

struct tmc2130_bus {
    spi_host_device_t spi_host;
    int en_pin;
    uint32_t spi_clock_hz;
    driver_state_t drivers[TMC2130_BUS_NUM_DRIVERS];

    bool initialized;
};

static struct tmc2130_bus current_bus = {
    .initialized = false,
};

// ============================================================================
// Internal static methods
// ============================================================================

static bool driver_id_is_valid(uint8_t driver_id) {
    return (current_bus.initialized) &&
           (driver_id < TMC2130_BUS_NUM_DRIVERS) &&
           (current_bus.drivers[driver_id].initialized);
}

/**
 * @brief Calculate TMC2130 current parameters based on desired RMS current
 * @param ma_rms Desired RMS current in milliamperes (mA)
 * @param vsense_high Output: true for VSENSE=1 (high sensitivity), false for VSENSE=0 (low sensitivity)
 * @param cs Output: Calculated Current Scale value (0-31)
 * @param actual_current Output: Actual current that will be achieved in mA
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parameters invalid
 * 
 * @note Based on TMCStepper library formula:
 *       CS = 32 * sqrt(2) * I_rms * (Rsense + 0.02) / V_fs - 1
 *       Automatically selects VSENSE based on calculated CS:
 *       - If CS < 16: uses VSENSE=1 (high sensitivity, Vfs=0.18V)
 *       - If CS >= 16: uses VSENSE=0 (low sensitivity, Vfs=0.32V)
 */
static esp_err_t tmc2130_calculate_current_params(
    uint16_t ma_rms,
    bool *vsense_high,
    uint8_t *cs,
    float *actual_current)
{
    if (vsense_high == NULL || cs == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const float RSENSE = 0.11f;             // External sense resistor value
    const float INTERNAL_RSENSE = 0.02f;    // Internal sense resistor (from datasheet)
    const float SQRT2 = 1.41421356f;
    
    // Try with VSENSE=0 first (low sensitivity, Vfs=0.32V)
    float V_fs = 0.32f;  // Vfs for VSENSE=0 (from TMCStepper)
    bool vsense_high_tmp = false;
    
    // Calculate CS: 32 * sqrt(2) * I_rms * (Rsense + 0.02) / V_fs - 1
    float current_A = ma_rms / 1000.0f;
    float cs_float = 32.0f * SQRT2 * current_A * (RSENSE + INTERNAL_RSENSE) / V_fs - 1.0f;
    int16_t cs_tmp = (int16_t)(cs_float + 0.5f); // Round to nearest integer
    
    ESP_LOGD(TAG, "VSENSE=0 preliminary CS = %d (%.2f) for %dmA", cs_tmp, cs_float, ma_rms);
    
    // If CS is too low (< 16), switch to VSENSE=1 (high sensitivity, Vfs=0.18V)
    if (cs_tmp < 16) {
        vsense_high_tmp = true;
        V_fs = 0.18f;  // Vfs for VSENSE=1
        cs_float = 32.0f * SQRT2 * current_A * (RSENSE + INTERNAL_RSENSE) / V_fs - 1.0f;
        cs_tmp = (int16_t)(cs_float + 0.5f);
        ESP_LOGD(TAG, "VSENSE=1 preliminary CS = %d (%.2f) for %dmA", cs_tmp, cs_float, ma_rms);
    }
    
    // Clamp CS to valid range (0-31)
    if (cs_tmp > 31) {
        cs_tmp = 31;
        ESP_LOGD(TAG, "Requested current %dmA too high, limited to CS=31", ma_rms);
    }
    if (cs_tmp < 0) {
        cs_tmp = 0;
        ESP_LOGD(TAG, "Requested current %dmA too low, set to CS=0", ma_rms);
    }
    
    // Calculate actual current for the selected VSENSE and CS
    float actual_current_tmp = (cs_tmp + 1) * V_fs / (32.0f * SQRT2 * (RSENSE + INTERNAL_RSENSE)) * 1000.0f;
    
    // Set output parameters
    *vsense_high = vsense_high_tmp;
    *cs = (uint8_t)cs_tmp;
    
    if (actual_current != NULL) {
        *actual_current = actual_current_tmp;
    }

    return ESP_OK;
}

/**
 * @brief Configure TMC2130 PWMCONF register for stealthChop mode
 * @param driver_id Driver number 0..2
 * @param pwm_autoscale Enable automatic PWM amplitude scaling (recommended)
 * @param freewheeling_mode: Freewheeling mode (0-3)
 *        0: Normal operation
 *        1: Freewheeling
 *        2: Coil shorted using LS drivers
 *        3: Coil shorted using HS drivers
 * @return ESP_OK on success, error code otherwise
 * 
 * @note Optimized for 0.11Ω sense resistors and typical NEMA motors
 * @details Register description:
 *      --------------------------------------------
        0X70: PWMCONF – VOLTAGE MODE PWM STEALTHCHOP
        --------------------------------------------
        
        bit >21 reserved, set to 0

        bit #21,20
        bit name(s): freewheel1, freewheel0
        #define TMC2130_FREEWHEEL_MASK                0x00300000
        #define TMC2130_FREEWHEEL_SHIFT               20
        function/comments: Allows different standstill modes.
            Stand still option when motor current setting is zero (IHOLD=0).
            %00: Normal operation
            %01: Freewheeling
            %10: Coil shorted using LS drivers
            %11: Coil shorted using HS drivers

        bit #19
        bit name(s): pwm_symmetric.
        #define TMC2130_PWM_SYMMETRIC_MASK            0x00080000
        #define TMC2130_PWM_SYMMETRIC_SHIFT           19
        function/comments: Force symmetric PWM
            0: The PWM value may change within each PWM cycle (standard mode)
            1: A symmetric PWM cycle is enforced

        bit #18
        bit name(s): pwm_autoscale
        #define TMC2130_PWM_AUTOSCALE_MASK            0x00040000
        #define TMC2130_PWM_AUTOSCALE_SHIFT           18
        function/comments: PWM automatic amplitude scaling
            0: User defined PWM amplitude. The current settings have no influence.
            1: Enable automatic current control
               Attention: When using a user defined sine wave table,
               the amplitude of this sine wave table should not be less than 244.
               Best results are obtained with 247 to 252 as peak values.

        bit #17,16
        bit name(s): pwm_freq1,pwm_freq0
        #define TMC2130_PWM_FREQ_MASK                 0x00030000
        #define TMC2130_PWM_FREQ_SHIFT                16
        function/comments: PWM frequency selection
            %00: fPWM=2/1024 f(CLK)
            %01: fPWM=2/683 f(CLK)
            %10: fPWM=2/512 f(CLK)
            %11: fPWM=2/410 f(CLK)
        #define TMC2130_PWM_GRAD_MASK                 0x0000FF00
        #define TMC2130_PWM_GRAD_SHIFT                8
        bit #15,14,13,12,11,10,9,8
        bit name(s): PWM_GRAD
        function/comments: User defined amplitude (gradient) or regulation loop gradient
            if bit #18 pwm_autoscale=0: Velocity dependent gradient for PWM amplitude:
                                        PWM_GRAD * 256 / TSTEP is added to PWM_AMPL,
            if bit #18 pwm_autoscale=1: User defined maximum PWM amplitude change per half wave (1 to 15).

        bit #7,6,5,4,3,2,1,0
        bit name(s): PWM_AMPL
        #define TMC2130_PWM_AMPL_MASK                 0x000000FF
        #define TMC2130_PWM_AMPL_SHIFT                0
        function/comments: User defined amplitude (offset)
            if bit #18 pwm_autoscale=0: User defined PWM amplitude offset (0-255)
                                        The resulting amplitude (limited to 0…255) is:
                                        PWM_AMPL + PWM_GRAD * 256 / TSTEP
            if bit #18 pwm_autoscale=1: User defined maximum PWM amplitude when switching back from current chopper
                                        mode to voltage PWM mode (switch over velocity defined by TPWMTHRS).
                                        Do not set too low values, as the regulation cannot measure the current
                                        when the actual PWM value goes below a setting specific value.
                                        Settings above 0x40 recommended.

 */
esp_err_t tmc2130_configure_pwmconf(uint8_t driver_id, bool pwm_autoscale, uint8_t freewheeling_mode) {
    uint32_t pwmconf = 0; // Reserved bits [31:22] - Must be 0
    
    pwmconf |= ((freewheeling_mode << TMC2130_FREEWHEEL_SHIFT) & TMC2130_FREEWHEEL_MASK);
    freewheeling_mode = ((pwmconf & TMC2130_FREEWHEEL_MASK) >> TMC2130_FREEWHEEL_SHIFT);

    static const uint32_t FRSH = 0;
    static const uint32_t SYSH = 0;
    
    pwmconf |= (FRSH << TMC2130_PWM_FREQ_SHIFT);        // PWM frequency selection: %00 = 2/1024 fCLK (~24.4kHz with 16MHz clock)    
    pwmconf |= (SYSH << TMC2130_PWM_SYMMETRIC_SHIFT);   // Force symmetric PWM: 0 = Standard mode (PWM value can change within cycle)    

    if (pwm_autoscale) {
        static const uint32_t AUSC = 1;
        static const uint32_t AMPL = 128;
        static const uint32_t GRAD = 12;
        
        pwmconf |= (AUSC << TMC2130_PWM_AUTOSCALE_SHIFT);   // Automatic PWM amplitude scaling: 1 = Enable automatic current control (RECOMMENDED)
        pwmconf |= (AMPL << TMC2130_PWM_AMPL_SHIFT);        // User defined maximum PWM amplitude: 0x80 (128) is recommended when PWM_AUTOSCALE=1
        pwmconf |= (GRAD << TMC2130_PWM_GRAD_SHIFT);        // Regulation loop gradient: 12 is good for 0.11Ω sense resistors (range 1-15)    
        
        ESP_LOGI(
            TAG,
            "Driver #%d PWMCONF: AUTOSCALE=%d, AMPL=%d, GRAD=%d, FREQ=%d, SYMM=%d, FREEWHEEL=%d",
            driver_id, AUSC, AMPL, GRAD, FRSH, SYSH, freewheeling_mode);
            
    } else {
        static const uint32_t AUSC = 0;                     // Manual PWM amplitude mode, user defines PWM amplitude directly
        static const uint32_t AMPL = 50;    
        static const uint32_t GRAD = 25;
       
        pwmconf |= (AUSC << TMC2130_PWM_AUTOSCALE_SHIFT);   // Automatic PWM amplitude scaling: 0 = Disable automatic current control
        pwmconf |= (AMPL << TMC2130_PWM_AMPL_SHIFT);        // Base PWM amplitude offset. Range: 0-255, typical: 40-80, 50 is good starting point for 0.11Ω
        pwmconf |= (GRAD << TMC2130_PWM_GRAD_SHIFT);        // Velocity dependent gradient: PWM_GRAD * 256 / TSTEP is added to PWM_AMPL, 25 is good default value.
        
        ESP_LOGI(
            TAG,
            "Driver #%d PWMCONF: AUTOSCALE=%d, AMPL=%d, GRAD=%d, FREQ=%d, SYMM=%d, FREEWHEEL=%d",
            driver_id, AUSC, AMPL, GRAD, FRSH, SYSH, freewheeling_mode);
    }
    
    esp_err_t res = tmc2130_writeRegister(driver_id, TMC2130_PWMCONF, pwmconf);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to write register PWMCONF: %s", esp_err_to_name(res));
    }

    return res;
}

/**
 * @brief Configure TMC2130 CHOPCONF register based on operating mode.
 * @param driver_id driver ID number 0..2
 * @param stealthchop true for stealthChop mode, false for SpreadCycle
 * @param microsteps Microstep resolution (1,2,4,8,16,32,64,128,256)
 * @param vsense_high true for VSENSE=1 (high sensitivity), false for VSENSE=0 (low sensitivity)
 * @return ESP_OK on success, error code otherwise
 * @details Register descriptions:

        0X6C: CHOPCONF – CHOPPER CONFIGURATION
        --------------------------------------
        bit #31
        bit name(s): -
        function/comments: -
        Reserved, set to 0

        bit #30
        bit name(s): diss2g
        #define TMC2130_DISS2G_MASK                   0x40000000
        #define TMC2130_DISS2G_SHIFT                  30
        function/comments: short to GND 0: Short to GND protection is on
        protection disable 1: Short to GND protection is disabled

        bit #29
        bit name(s): dedge
        #define TMC2130_DEDGE_MASK                    0x20000000
        #define TMC2130_DEDGE_SHIFT                   29
        function/comments: enable double edge step pulses
        1: Enable step impulse at each step edge to reduce step frequency requirement.

        bit #28
        bit name(s): intpol
        #define TMC2130_INTPOL_MASK                   0x10000000
        #define TMC2130_INTPOL_SHIFT                  28
        function/comments: interpolation to 256 microsteps
        1: The actual microstep resolution (MRES) becomes extrapolated to 256 microsteps for smoothest motor operation.

        bit #27,26,25,24
        bit name(s): mres3,mres2,mres1,mres0
        #define TMC2130_MRES_MASK                     0x0F000000
        #define TMC2130_MRES_SHIFT                    24
        function/comments: MRES micro step resolution
        %0000: Native 256 microstep setting.
        %0001 … %1000: 128, 64, 32, 16, 8, 4, 2, FULLSTEP
        Reduced microstep resolution for STEP/DIR operation.
        The resolution gives the number of microstep entries per sine quarter wave.
        The driver automatically uses microstep positions which result in a symmetrical wave, when choosing a lower microstep resolution.
        step width=2^MRES [microsteps]

        bit #23,22,21,20
        bit name(s): sync3,sync2,sync1,sync0
        #define TMC2130_VHIGHCHM_MASK                 0x00080000
        #define TMC2130_VHIGHCHM_SHIFT                19
        function/comments: SYNC: PWM synchronization clock
        This register allows synchronization of the chopper for both phases of a two-phase motor to avoid the occurrence of a beat, especially at low motor velocities. It is
        automatically switched off above VHIGH.
        %0000: Chopper sync function chopSync off
        %0001 … %1111: Synchronization with fSYNC = fCLK/(sync*64)
        Hint: Set TOFF to a low value, so that the chopper cycle is ended before the next sync 
        clock pulse occurs. Set for the double desired chopper frequency for chm=0, for the
        desired base chopper frequency for chm=1.

        bit #19
        bit name(s): vhighchm
        #define TMC2130_VHIGHCHM_MASK                 0x00080000
        #define TMC2130_VHIGHCHM_SHIFT                19
        function/comments: high velocity chopper mode
        This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded. This way, a higher velocity can be achieved. Can be combined with vhighfs=1. If set, the TOFF setting automatically becomes doubled during high velocity operation in order to avoid doubling of the chopper frequency.

        bit #18
        bit name(s): vhighfs
        #define TMC2130_VHIGHFS_MASK                  0x00040000
        #define TMC2130_VHIGHFS_SHIFT                 18
        function/comments: high velocity fullstep selection
        This bit enables switching to fullstep, when VHIGH is exceeded. Switching takes place only at 45° position. The fullstep target current uses the current value from the microstep table at the 45° position.

        bit #17
        bit name(s): vsense
        #define TMC2130_VSENSE_MASK                   0x00020000
        #define TMC2130_VSENSE_SHIFT                  17
        function/comments: sense resistor voltage based current scaling
        0: Low sensitivity, high sense resistor voltage
        1: High sensitivity, low sense resistor voltage

        bit #16,15
        bit name(s): tbl1,tbl0
        #define TMC2130_TBL_MASK                      0x00018000
        #define TMC2130_TBL_SHIFT                     15
        function/comments: TBL
        blank time select
        %00 … %11:
        Set comparator blank time to 16, 24, 36 or 54 clocks.
        Hint: %01 or %10 is recommended for most applications.

        bit #14
        bit name(s): chm
        #define TMC2130_CHM_MASK                      0x00004000
        #define TMC2130_CHM_SHIFT                     14
        function/comments: chopper mode
        0: Standard mode (SpreadCycle)
        1: Constant off time with fast decay time. Fast decay time is also terminated when the
        negative nominal current is reached. Fast decay is after on time.

        bit #13
        bit name(s): rndtf
        #define TMC2130_RNDTF_MASK                    0x00002000
        #define TMC2130_RNDTF_SHIFT                   13
        function/comments: random TOFF time
        0: Chopper off time is fixed as set by TOFF
        1: Random mode, TOFF is random modulated by dN(CLK)= -24 … +6 clocks.

        bit #12
        bit name(s): disfdcc
        #define TMC2130_DISFDCC_MASK                  0x00001000
        #define TMC2130_DISFDCC_SHIFT                 12
        function/comments: fast decay mode
        chm=1: disfdcc=1 disables current comparator usage for termination of the fast decay cycle.

        bit #11
        bit name(s): fd3
        #define TMC2130_TFD___MASK                    0x00000800
        #define TMC2130_TFD___SHIFT                   11
        function/comments: TFD [3]
        chm=1: MSB of fast decay time setting TFD

        bit #10,9,8,7
        bit name(s): hend3,hend2,hend1,hend0
        #define TMC2130_OFFSET_MASK                   0x00000780
        #define TMC2130_OFFSET_SHIFT                  7
        #define TMC2130_HEND_MASK                     0x00000780
        #define TMC2130_HEND_SHIFT                    7
        function/comments: HEND hysteresis low value: chm=0: %0000 … %1111: Hysteresis is -3, -2, -1, 0, 1, …, 12. (1/512 of this setting adds to current setting). This is the hysteresis value which becomes used for the hysteresis chopper.
        OFFSET sine wave offset: chm=1 %0000 … %1111: Offset is -3, -2, -1, 0, 1, …, 12. This is the sine wave offset and 1/512 of the value becomes added to the absolute value
        of each sine wave entry.

        bit #6,5,4
        bit name(s): hstrt2,hstrt1,hstrt0
        chm=0:
        #define TMC2130_HSTRT_MASK                    0x00000070
        #define TMC2130_HSTRT_SHIFT                   4
        function/comments: HSTRT hysteresis start value added to HEND
        %000 … %111: Add 1, 2, …, 8 to hysteresis low value HEND (1/512 of this setting adds to current setting) Attention: Effective HEND+HSTRT ≤ 16. Hint: Hysteresis decrement is done each 16 clocks.
        chm=1:
        #define TMC2130_TFD_2__0__MASK                0x00000070
        #define TMC2130_TFD_2__0__SHIFT               4
        function/comments: TFD [2..0] fast decay time setting. Fast decay time setting (MSB is here: fd3): %0000 … %1111: Fast decay time setting TFD with N(CLK)=32*TFD (%0000: slow decay only)/.

        bit #3,2,1,0
        bit name(s): toff3,toff2,toff1,toff0
        #define TMC2130_TOFF_MASK                     0x0000000F
        #define TMC2130_TOFF_SHIFT                    0
        function/comments: TOFF off time and driver enable
        Off time setting controls duration of slow decay phase. N(CLK)=24+32*TOFF
        %0000: Driver disable, all bridges off
        %0001: 1 – use only with TBL ≥ 2
        %0010 … %1111: 2 … 15
 */
esp_err_t tmc2130_configure_chopconf(uint8_t driver_id, bool stealthchop, uint8_t microsteps, bool vsense_high) {
    uint32_t chopconf = 0;
    
    // Convert microsteps to MRES value
    // MRES: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=1
    uint8_t mres = 0;
    switch(microsteps) {
        case 0:   mres = 0; break; // 0 = 256
        case 128: mres = 1; break;
        case 64:  mres = 2; break;
        case 32:  mres = 3; break;
        case 16:  mres = 4; break;
        case 8:   mres = 5; break;
        case 4:   mres = 6; break;
        case 2:   mres = 7; break;
        case 1:   mres = 8; break;
        default:  mres = 4; // Default to 1/16 microsteps
    }
    
    // Common settings for both modes with Rsense = 0.11Ω
    chopconf |= (mres << TMC2130_MRES_SHIFT);      // Microstep resolution
    chopconf |= (1 << TMC2130_INTPOL_SHIFT);       // Enable 256x interpolation (smoother operation)
    chopconf |= ((vsense_high ? 1 : 0) << TMC2130_VSENSE_SHIFT);    // VSENSE bit from parameter
    chopconf |= (0 << TMC2130_DISS2G_SHIFT);       // DISS2G=0 - Enable short to GND protection
    chopconf |= (0 << TMC2130_DEDGE_SHIFT);        // DEDGE=0 - Single edge step pulses
    chopconf |= (0 << TMC2130_VHIGHCHM_SHIFT);     // VHIGHCHM=0 - No automatic mode switching
    chopconf |= (0 << TMC2130_VHIGHFS_SHIFT);      // VHIGHFS=0 - No fullstep switching at high velocity
    chopconf |= (0 << TMC2130_RNDTF_SHIFT);        // RNDTF=0 - Fixed TOFF time, no random modulation
    
    if (stealthchop) {
        /**
         * stealthChop mode configuration
         * - Ultra-quiet operation
         * - No hysteresis needed
         * - Shorter off-time for better high-speed performance
         * - Longer blank time for reliable current measurement
         */
        chopconf |= (0 << TMC2130_CHM_SHIFT);       // CHM=0 (SpreadCycle chopper, stealthChop enabled in GCONF)
        chopconf |= (2 << TMC2130_TBL_SHIFT);       // TBL=2 (36 clocks) - More blank time for accurate measurement
        chopconf |= (3 << TMC2130_TOFF_SHIFT);      // TOFF=3 - Shorter off-time for stealthChop
        chopconf |= (0 << TMC2130_HSTRT_SHIFT);     // HSTRT=0 - No hysteresis start delay
        chopconf |= (0 << TMC2130_HEND_SHIFT);      // HEND=0 - No hysteresis
        chopconf |= (0 << TMC2130_DISFDCC_SHIFT);   // DISFDCC=0 - Enable fast decay comparator (not used in stealthChop)
        
        ESP_LOGI(
            TAG,
            "Driver #%d: CHOPCONF: stealthChop mode, MRES=%d (1/%d steps)",
            driver_id,
            mres,
            microsteps == 0 ? 256: microsteps);
    } else {
        /**
         * SpreadCycle mode configuration
         * - Optimal settings for 0.11Ω sense resistors
         * - Good middle ground between smoothness and power
         * - Recommended for most applications
         */
        chopconf |= (0 << TMC2130_CHM_SHIFT);       // CHM=0 (SpreadCycle mode)
        chopconf |= (1 << TMC2130_TBL_SHIFT);       // TBL=1 (24 clocks) - Standard blank time
        chopconf |= (7 << TMC2130_TOFF_SHIFT);      // TOFF=5 - Standard off-time for SpreadCycle (increase to 7)
        chopconf |= (7 << TMC2130_HSTRT_SHIFT);     // HSTRT=5 - Hysteresis start delay (increase to 7)
        chopconf |= (6 << TMC2130_HEND_SHIFT);      // HEND=2 - Hysteresis low value (increase to 6)
        chopconf |= (0 << TMC2130_DISFDCC_SHIFT);   // DISFDCC=0 - Enable fast decay comparator
        
        ESP_LOGI(
            TAG,
            "Driver #%d: CHOPCONF: SpreadCycle mode, MRES=%d (1/%d steps), HSTRT=5, HEND=2",
            driver_id,
            mres,
            microsteps == 0 ? 256: microsteps);
    }
    
    // Write CHOPCONF register to driver
    esp_err_t res = tmc2130_writeRegister(driver_id, TMC2130_CHOPCONF, chopconf);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to write register CHOPCONF: %s", esp_err_to_name(res));
        return res;
    }
    
    return ESP_OK;
}

/**
 * @brief Configure TMC2130 IHOLD_IRUN register
 * @param driver_id driver ID number 0..2
 * @param cs_run Current Scale value for running current (0-31)
 * @param cs_hold Current Scale value for hold current (0-31)
 * @param hold_delay Delay before reducing to hold current (0-15, default 5)
 * @return ESP_OK on success, error code otherwise
 * 
 * @note This function only writes the IHOLD_IRUN register.
 *       VSENSE must be configured separately in CHOPCONF.
 */
esp_err_t tmc2130_configure_ihold_irun(
    uint8_t driver_id, 
    uint8_t cs_run, 
    uint8_t cs_hold,
    uint8_t hold_delay)
{
    // Validate parameters
    if (cs_run > 31 || cs_hold > 31) {
        ESP_LOGE(TAG, "Driver #%d: CS values must be 0-31 (run=%d, hold=%d)", 
                 driver_id, cs_run, cs_hold);
        return ESP_ERR_INVALID_ARG;
    }

    if (hold_delay > 15) {
        hold_delay = 15;
        ESP_LOGW(TAG, "Driver #%d: Hold delay too high, limited to 15", driver_id);
    }

    uint32_t ihold_irun = (cs_run << 0) |      // IRUN
                          (cs_hold << 8) |     // IHOLD
                          (hold_delay << 16);  // IHOLDDELAY
    
    /**
     * IHOLD_IRUN register format:
     * Bits 0-4:   IRUN - Running current (0-31)
     * Bits 8-12:  IHOLD - Hold current (0-31)
     * Bits 16-19: IHOLDDELAY - Delay before reducing to hold current (0-15)
     */
    ihold_irun = (cs_run  << 0)  |   // IRUN
                 (cs_hold << 8)  |   // IHOLD
                 (hold_delay << 16); // IHOLDDELAY
    
    esp_err_t res = tmc2130_writeRegister(driver_id, TMC2130_IHOLD_IRUN, ihold_irun);
    if (res != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Driver #%d: Failed to write register IHOLD_IRUN: %s",
            driver_id,
            esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(
        TAG,
        "Driver #%d: CS_RUN=%d, CS_HOLD=%d, hold_delay=%d", 
        driver_id,
        cs_run,
        cs_hold,
        hold_delay);

    return ESP_OK;
}

/**
 * @brief Configure TMC2130 GCONF register based on operating mode.
 * @param driver_id driver ID number 0..2
 * @param stealthchop true for stealthChop mode, false for SpreadCycle
 * @param invert_dir true for inverting shaft rotation direction, otherwise false
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t tmc2130_configure_gconf(uint8_t driver_id, bool stealthchop, bool invert_dir) {
    uint32_t gconf_value = 0;

    gconf_value &= ~TMC2130_I_SCALE_ANALOG_MASK;    // disable analog current scale, use digital from registers 
    gconf_value &= ~TMC2130_INTERNAL_RSENSE_MASK;   // disable internal Rsense resistors, use external 0.11Ω resistors.
    gconf_value = stealthchop                       
        ? gconf_value | TMC2130_EN_PWM_MODE_MASK    // enable StealthChop if needed
        : gconf_value & ~TMC2130_EN_PWM_MODE_MASK;
    gconf_value &= ~TMC2130_ENC_COMMUTATION_MASK;   // encoder commutation disable
    gconf_value = invert_dir                        
        ? gconf_value | TMC2130_SHAFT_MASK          // invert shaft direction if needed
        : gconf_value & ~TMC2130_SHAFT_MASK;
    gconf_value &= ~TMC2130_DIAG0_ERROR_MASK;       // do not use Diag0 for driver error
    gconf_value &= ~TMC2130_DIAG0_OTPW_MASK;        // do not use Diag0 for driver overheat warning

    esp_err_t res = tmc2130_writeRegister(driver_id, TMC2130_GCONF, gconf_value);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to write register GCONF: %s", esp_err_to_name(res));
    }
    
    return res;
}

// ============================================================================
// SPI main method implementation for Trinamic TMC API
// ============================================================================
esp_err_t tmc2130_readWriteSPI(uint16_t driver_id, uint8_t *data, size_t data_length) {
    /// TODO: Check whats wrong here:
    // if (driver_id_is_valid(driver_id)) {
    //     ESP_LOGE(TAG, "Initialize SPI bus first (call tmc2130_init())");
    //     return;
    // }
    
    spi_transaction_t t = {
        .length = data_length * 8,
        .tx_buffer = data,  // TMC2130 uses same buffer:
        .rx_buffer = data,  // transmitted data gets replaced by response.
    };
    

    gpio_set_level(current_bus.drivers[driver_id].cs_pin, 0);   // set CS low

    esp_err_t ret = spi_device_transmit(                                  // execute transaction
        current_bus.drivers[driver_id].spi_handle, &t);

    gpio_set_level(current_bus.drivers[driver_id].cs_pin, 1);   // set CS high
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
    }
    
    esp_rom_delay_us(1);

    return ESP_OK;
}

// ============================================================================
// Initialize single driver (internal)
// ============================================================================

// Common SPI device config (for all TMC2130 drivers)
static const spi_device_interface_config_t spi_dev_config = {
    .clock_speed_hz = -1,   // to be initialized from bus factual value
    .spics_io_num = -1,     // to be initialized from config
    .mode = 3,              // bit pair combination CPOL=1, CPHA=1 (always for TMC2130)
    .queue_size = 1,
    .flags = SPI_DEVICE_NO_DUMMY,
};

static esp_err_t init_single_driver(uint8_t driver_id, const tmc2130_driver_config_t* driver_config) {
    spi_device_interface_config_t devcfg = spi_dev_config;
    devcfg.clock_speed_hz = current_bus.spi_clock_hz;
    devcfg.spics_io_num = driver_config->cs_pin;
    
    esp_err_t ret = spi_bus_add_device(current_bus.spi_host, &devcfg, &current_bus.drivers[driver_id].spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to add SPI device for driver #%d (CS pin %d): %s",
            driver_id,
            driver_config->cs_pin,
            esp_err_to_name(ret));
        return ret;
    }
    
    current_bus.drivers[driver_id].cs_pin = driver_config->cs_pin;
    current_bus.drivers[driver_id].step_pin = driver_config->step_pin;
    current_bus.drivers[driver_id].dir_pin = driver_config->dir_pin;
    
    current_bus.drivers[driver_id].microsteps = driver_config->microsteps;
    current_bus.drivers[driver_id].run_current_ma = driver_config->run_current_ma 
            ? driver_config->run_current_ma
            : 1000;
    current_bus.drivers[driver_id].hold_current_ma = driver_config->hold_current_ma 
            ? driver_config->hold_current_ma
            : 600;
    current_bus.drivers[driver_id].hold_delay_ms = 1;

    current_bus.drivers[driver_id].stealthchop = driver_config->stealthchop;
    current_bus.drivers[driver_id].invert_dir = driver_config->invert_dir;

    current_bus.drivers[driver_id].initialized = true;
    
    ESP_LOGI(
        TAG,
        "Driver #%d: SPI device added (CS=%d, STEP=%d, DIR=%d)", 
        driver_id,
        driver_config->cs_pin,
        driver_config->step_pin,
        driver_config->dir_pin);
    
    return ESP_OK;
}

static esp_err_t configure_single_driver(uint8_t driver_id) {
    /// TODO: Check whats wrong here:
    // if (!driver_id_is_valid(driver_id)) {
    //     return ESP_ERR_INVALID_ARG;
    // }
    
    driver_state_t* driver = &current_bus.drivers[driver_id];
    esp_err_t res;
    
    // Check connection
    uint32_t ioin;
    res = tmc2130_readRegister(driver_id, TMC2130_IOIN, &ioin);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to read register IOIN: %s", esp_err_to_name(res));
        return res;
    }
    
    uint32_t version = tmc2130_fieldExtract(ioin, TMC2130_VERSION_FIELD);
    
    if (version != 0x11 && version != 0x21) {
        ESP_LOGW(TAG, "Driver #%d: Unexpected TMC2130 version 0x%02X", driver_id, version);
    } else {
        ESP_LOGI(TAG, "Driver #%d: TMC2130 OK: version v0x%02X, ioin: 0x%08lX.", driver_id, version, ioin);
    }
    
    // Read GSTAT register to reset the errors
    uint32_t gstat;
    res = tmc2130_readRegister(driver_id, TMC2130_GSTAT, &gstat);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to read register GSTAT: %s", esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(TAG, "Initial GSTAT: 0x%08lX", gstat);
    if (tmc2130_fieldExtract(gstat, TMC2130_DRV_ERR_FIELD)) {
        ESP_LOGW(TAG, "Driver #%d error flag was set: TMC2130_DRV_ERR_FIELD", driver_id);
    }
    if (tmc2130_fieldExtract(gstat, TMC2130_UV_CP_FIELD)) {
        ESP_LOGW(TAG, "Driver #%d undervoltage on charge pump detected: TMC2130_UV_CP_FIELD", driver_id);
    }

    res = tmc2130_configure_gconf(
        driver_id,
        current_bus.drivers[driver_id].stealthchop,
        current_bus.drivers[driver_id].invert_dir);
    if (res != ESP_OK) {
        return res;
    }

    // Calculate current parameters based on desired RMS current
    // This determines VSENSE and CS values
    bool vsense_high;
    uint8_t cs_run;
    float actual_current;
    
    res = tmc2130_calculate_current_params(
        driver->run_current_ma,   // Desired RMS current in mA
        &vsense_high,             // Output: VSENSE setting
        &cs_run,                  // Output: Current Scale for running
        &actual_current           // Output: Actual current that will be achieved
    );
    if (res != ESP_OK) {
        return res;
    }
    else
    {
        ESP_LOGW(
            TAG, 
            "Driver #%d current calculation: %dmA RMS -> CS=%d, VSENSE=%s, actual current: %.0fmA",
            driver_id,
            driver->run_current_ma,
            cs_run,
            vsense_high ? "HIGH(1)" : "LOW(0)",
            actual_current);
    }

    float hold_multiplier = (float)driver->hold_current_ma / (float)driver->run_current_ma;
    if (hold_multiplier > 1.0f) hold_multiplier = 1.0f;
    
    uint8_t cs_hold = (uint8_t)(cs_run * hold_multiplier);
    if (cs_hold > 31) cs_hold = 31;
    
    res = tmc2130_configure_ihold_irun(
        driver_id,
        cs_run,
        cs_hold,
        driver->hold_delay_ms);
    if (res != ESP_OK) {
        return res;
    }
    
    res = tmc2130_configure_chopconf(
        driver_id,
        current_bus.drivers[driver_id].stealthchop,
        current_bus.drivers[driver_id].microsteps,
        vsense_high); // Pass calculated VSENSE
    if (res != ESP_OK) { 
        return res;
    }

    res = tmc2130_configure_pwmconf(driver_id, true, 0);
    if (res != ESP_OK) {
        return res;
    }    
    
    /// TODO: add config for TMC2130_COOLCONF
        
    ESP_LOGI(TAG, "TMC2130 initialized successfully");
    return ESP_OK;
}

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize a TMC2130 stepper driver bus (SPI)
 *
 * This function sets up the SPI bus for communication with one or more TMC2130 stepper motor drivers.
 * It initializes the SPI host controller, configures bus parameters, and prepares driver structures.
 * Must be called before using any TMC2130 driver functions.
 *
 * @param config Pointer to bus configuration structure
 * @return
 *      - ESP_OK: Bus initialized successfully
 *      - ESP_ERR_INVALID_ARG: Invalid configuration or NULL pointer
 *      - ESP_ERR_NO_MEM: Memory allocation failed  
 *      - ESP_ERR_NOT_FOUND: No available SPI host
 *      - Other SPI-related errors from spi_bus_initialize()
 *
 * @note Only one bus instance per SPI host is supported.
 * @note The bus must be initialized before adding individual drivers.
 */
esp_err_t tmc2130_init(const tmc2130_config_t *config) {
#if TMC2130_CACHE == 1 && TMC2130_ENABLE_TMC_CACHE == 1
    tmc2130_initCache();
#endif

    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (current_bus.initialized) {
        ESP_LOGE(TAG, "Bus already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&current_bus, 0, sizeof(struct tmc2130_bus));
    current_bus.spi_host = config->spi_host;
    current_bus.spi_clock_hz = config->spi_clock_hz;
    current_bus.en_pin = config->en_pin;
    current_bus.initialized = false;
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = config->mosi_pin,
        .miso_io_num = config->miso_pin,
        .sclk_io_num = config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TMC2130_BUS_NUM_DRIVERS * 64,
    };
    
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(
        TAG,
        "SPI bus initialized (host: %d, clock: %d Hz)", 
        config->spi_host,
        config->spi_clock_hz);
    
    uint64_t gpio_mask = (1ULL << config->en_pin);
    
    for (int i = 0; i < TMC2130_BUS_NUM_DRIVERS; i++) {
        gpio_mask |= (1ULL << config->drivers[i].cs_pin);
        gpio_mask |= (1ULL << config->drivers[i].step_pin);
        gpio_mask |= (1ULL << config->drivers[i].dir_pin);
    }
    
    gpio_config_t io_conf = {
        .pin_bit_mask = gpio_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    gpio_set_level(config->en_pin, 1); // disable drivers after initialization
    
    for (int i = 0; i < TMC2130_BUS_NUM_DRIVERS; i++) {
        ret = init_single_driver(i, &config->drivers[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize driver #%d: %s", i, esp_err_to_name(ret));
            
            for (int j = 0; j < i; j++) {
                if (current_bus.drivers[j].spi_handle != NULL) {
                    spi_bus_remove_device(current_bus.drivers[j].spi_handle);
                }
            }

            return ret;
        }
    }
    
    for (int i = 0; i < TMC2130_BUS_NUM_DRIVERS; i++) {
        ret = configure_single_driver(i);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Driver #%d configuration may have issues: %s", i, esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    current_bus.initialized = true;
    ESP_LOGI(TAG, "TMC2130 bus initialized with %d driver(s)", TMC2130_BUS_NUM_DRIVERS);
    
    return ESP_OK;
}

esp_err_t tmc2130_enable(bool enable) {
    if (!current_bus.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(current_bus.en_pin, enable ? 0 : 1);
    return ESP_OK;
}

void tmc2130_move_steps(uint8_t driver_id, int32_t steps, uint32_t delay_us) {
    gpio_set_level(current_bus.drivers[driver_id].dir_pin, (steps >= 0));
    steps = abs(steps);
    
    for (int32_t i = 0; i < steps; i++) {
        gpio_set_level(current_bus.drivers[driver_id].step_pin, 1);
        esp_rom_delay_us(2);
        gpio_set_level(current_bus.drivers[driver_id].step_pin, 0);
        esp_rom_delay_us(2);

        if (i % 100 == 0) {
            vTaskDelay(1);
        }

        if (delay_us > 4) {
            esp_rom_delay_us(delay_us - 4);
        }
    }
}

esp_err_t tmc2130_read_status(uint8_t driver_id, uint32_t *value) {
    return tmc2130_readRegister(driver_id, TMC2130_DRV_STATUS, value);
}

bool tmc2130_is_error(uint8_t driver_id) {
    uint32_t gstat;
    esp_err_t res = tmc2130_readRegister(driver_id, TMC2130_GSTAT, &gstat);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Driver #%d: Failed to read register GSTAT: %s", esp_err_to_name(res));
        return true;
    }
    
    return tmc2130_fieldExtract(gstat, TMC2130_DRV_ERR_FIELD) != 0;
}

void tmc2130_log_drv_status(uint8_t driver_id, uint32_t status) {
    if (status == 0) return;
    
    uint16_t sg_result = (status & TMC2130_SG_RESULT_MASK) >> TMC2130_SG_RESULT_SHIFT;
    uint8_t cs_actual = (status & TMC2130_CS_ACTUAL_MASK) >> TMC2130_CS_ACTUAL_SHIFT;
    
    char flags[128] = {0};
    char *ptr = flags;
    
    if (status & TMC2130_STALLGUARD_MASK) ptr += sprintf(ptr, "SG ");
    if (status & TMC2130_OT_MASK) ptr += sprintf(ptr, "OT! ");
    if (status & TMC2130_OTPW_MASK) ptr += sprintf(ptr, "OTPW ");
    if (status & TMC2130_S2GA_MASK) ptr += sprintf(ptr, "S2GA ");
    if (status & TMC2130_S2GB_MASK) ptr += sprintf(ptr, "S2GB ");
    if (status & TMC2130_OLA_MASK) ptr += sprintf(ptr, "OLA ");
    if (status & TMC2130_OLB_MASK) ptr += sprintf(ptr, "OLB ");
    if (status & TMC2130_STST_MASK) ptr += sprintf(ptr, "STST");
    
    ESP_LOGW(TAG, "DRV%d: 0x%08lX | SG=%3d | CS=%2d | Flags: %s", 
             driver_id, status, sg_result, cs_actual, flags);
}

// ============================================================================
// Public Runtime configuration setters (dynamic updates)
// ============================================================================

/**
 * @brief Set microstep resolution for a driver (runtime).
 * @param driver_id driver index (0..TMC2130_BUS_NUM_DRIVERS-1)
 * @param microsteps_code code: 0=256, 1=1, 2=2, 4,8,16,32,64,128
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_microsteps(uint8_t driver_id, uint8_t microsteps_code)
{
    if (!driver_id_is_valid(driver_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert code to actual microstep value
    uint8_t actual_microsteps;
    switch (microsteps_code) {
        case 0:  actual_microsteps = 256; break;
        case 1:  actual_microsteps = 1; break;
        default:
            if (microsteps_code == 2 || microsteps_code == 4 || microsteps_code == 8 ||
                microsteps_code == 16 || microsteps_code == 32 || microsteps_code == 64 ||
                microsteps_code == 128) {
                actual_microsteps = microsteps_code;
            } else {
                ESP_LOGE(TAG, "Invalid microsteps code: %d", microsteps_code);
                return ESP_ERR_INVALID_ARG;
            }
            break;
    }

    // Update stored value
    current_bus.drivers[driver_id].microsteps = actual_microsteps;

    // Need current vsense_high and stealthchop to reconfigure CHOPCONF
    bool vsense_high;
    uint8_t cs_run;
    float dummy;
    esp_err_t err = tmc2130_calculate_current_params(
        current_bus.drivers[driver_id].run_current_ma,
        &vsense_high, &cs_run, &dummy);
    if (err != ESP_OK) return err;

    err = tmc2130_configure_chopconf(driver_id,
                                     current_bus.drivers[driver_id].stealthchop,
                                     actual_microsteps,
                                     vsense_high);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update CHOPCONF for microsteps");
    }
    return err;
}

/**
 * @brief Set run current (RMS) for a driver (runtime).
 * @param driver_id driver index
 * @param ma desired current in milliamperes
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_run_current(uint8_t driver_id, uint16_t ma)
{
    if (!driver_id_is_valid(driver_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update stored value
    current_bus.drivers[driver_id].run_current_ma = ma;

    // Recalculate CS and VSENSE
    bool vsense_high;
    uint8_t cs_run;
    float actual_current;
    esp_err_t err = tmc2130_calculate_current_params(ma, &vsense_high, &cs_run, &actual_current);
    if (err != ESP_OK) return err;

    // Calculate hold current CS based on stored hold current
    float hold_multiplier = (float)current_bus.drivers[driver_id].hold_current_ma / (float)ma;
    if (hold_multiplier > 1.0f) hold_multiplier = 1.0f;
    uint8_t cs_hold = (uint8_t)(cs_run * hold_multiplier);
    if (cs_hold > 31) cs_hold = 31;

    // Update IHOLD_IRUN register (CS_RUN and CS_HOLD)
    err = tmc2130_configure_ihold_irun(driver_id, cs_run, cs_hold,
                                       current_bus.drivers[driver_id].hold_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update IHOLD_IRUN for run current");
        return err;
    }

    // Update CHOPCONF because VSENSE might have changed
    err = tmc2130_configure_chopconf(driver_id,
                                     current_bus.drivers[driver_id].stealthchop,
                                     current_bus.drivers[driver_id].microsteps,
                                     vsense_high);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update CHOPCONF for new VSENSE");
    }
    return err;
}

/**
 * @brief Set hold current (RMS) for a driver (runtime).
 * @param driver_id driver index
 * @param ma desired hold current in milliamperes
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_hold_current(uint8_t driver_id, uint16_t ma)
{
    if (!driver_id_is_valid(driver_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update stored value
    current_bus.drivers[driver_id].hold_current_ma = ma;

    // Get current CS_RUN from run current
    bool vsense_high;
    uint8_t cs_run;
    float dummy;
    esp_err_t err = tmc2130_calculate_current_params(
        current_bus.drivers[driver_id].run_current_ma,
        &vsense_high, &cs_run, &dummy);
    if (err != ESP_OK) return err;

    float hold_multiplier = (float)ma / (float)current_bus.drivers[driver_id].run_current_ma;
    if (hold_multiplier > 1.0f) hold_multiplier = 1.0f;
    uint8_t cs_hold = (uint8_t)(cs_run * hold_multiplier);
    if (cs_hold > 31) cs_hold = 31;

    err = tmc2130_configure_ihold_irun(driver_id, cs_run, cs_hold,
                                       current_bus.drivers[driver_id].hold_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update IHOLD_IRUN for hold current");
    }
    return err;
}

/**
 * @brief Enable/disable StealthChop mode for a driver (runtime).
 * @param driver_id driver index
 * @param enable true = StealthChop, false = SpreadCycle
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_stealthchop(uint8_t driver_id, bool enable)
{
    if (!driver_id_is_valid(driver_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update stored value
    current_bus.drivers[driver_id].stealthchop = enable;

    // Update GCONF register
    esp_err_t err = tmc2130_configure_gconf(driver_id, enable,
                                            current_bus.drivers[driver_id].invert_dir);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update GCONF for stealthchop");
        return err;
    }

    // Update CHOPCONF (different chopper settings for stealthchop/spreadcycle)
    bool vsense_high;
    uint8_t cs_run;
    float dummy;
    err = tmc2130_calculate_current_params(
        current_bus.drivers[driver_id].run_current_ma,
        &vsense_high, &cs_run, &dummy);
    if (err != ESP_OK) return err;

    err = tmc2130_configure_chopconf(driver_id, enable,
                                     current_bus.drivers[driver_id].microsteps,
                                     vsense_high);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update CHOPCONF for stealthchop");
    }
    return err;
}

/**
 * @brief Invert direction for a driver (runtime).
 * @param driver_id driver index
 * @param invert true = invert direction, false = normal
 * @return ESP_OK on success
 */
esp_err_t tmc2130_set_invert_dir(uint8_t driver_id, bool invert)
{
    if (!driver_id_is_valid(driver_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update stored value
    current_bus.drivers[driver_id].invert_dir = invert;

    // Update GCONF register only
    esp_err_t err = tmc2130_configure_gconf(driver_id,
                                            current_bus.drivers[driver_id].stealthchop,
                                            invert);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update GCONF for invert direction");
    }
    return err;
}
