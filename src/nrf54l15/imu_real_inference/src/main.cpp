#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <version.h>

// --- Edge Impulse SDK includes ---
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

LOG_MODULE_REGISTER(ei_lsm6dso_example, LOG_LEVEL_INF);


// --- Constants for 104Hz sampling rate ---
#define SAMPLING_RATE_HZ 104
#define SAMPLING_PERIOD_MS (1000 / SAMPLING_RATE_HZ)

// Sensitivity for the accelerometer at a Full Scale of ±2g. Unit: g/LSB
#define LSM6DSO_SENSITIVITY_G (0.000061f)

#if EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3
#error "This implementation assumes 3-axis accelerometer data."
#endif
#define NUM_SAMPLES_PER_INFERENCE (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)

// --- LSM6DSO I2C address and register definitions ---
#define LSM6DSO_I2C_ADDR    0x6A
#define LSM6DSO_REG_WHO_AM_I 0x0F
#define LSM6DSO_WHO_AM_I_VAL 0x6A
#define LSM6DSO_REG_CTRL1_XL 0x10
#define LSM6DSO_REG_CTRL2_G  0x11
#define LSM6DSO_REG_OUTX_L_XL 0x28

// --- Data buffers ---
static float features_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// --- Helper functions for I2C communication ---
static int lsm6dso_i2c_reg_write_byte(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value) {
    uint8_t tx_buf[2] = {reg_addr, value};
    return i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), LSM6DSO_I2C_ADDR);
}
static int lsm6dso_i2c_reg_read_byte(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *value) {
    return i2c_reg_read_byte(i2c_dev, LSM6DSO_I2C_ADDR, reg_addr, value);
}
static int lsm6dso_i2c_reg_read_bytes(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    return i2c_burst_read(i2c_dev, LSM6DSO_I2C_ADDR, reg_addr, data, len);
}

// --- LSM6DSO driver core functionality ---
static int lsm6dso_init(const struct device *i2c_dev) {
    uint8_t who_am_i = 0;
    int ret;
    ret = lsm6dso_i2c_reg_read_byte(i2c_dev, LSM6DSO_REG_WHO_AM_I, &who_am_i);
    if (ret != 0 || who_am_i != LSM6DSO_WHO_AM_I_VAL) {
        LOG_ERR("LSM6DSO not found. WHO_AM_I: 0x%02x", who_am_i);
        return -ENODEV;
    }
    LOG_INF("LSM6DSO WHO_AM_I check passed.");
    ret = lsm6dso_i2c_reg_write_byte(i2c_dev, LSM6DSO_REG_CTRL1_XL, 0x40); // 104Hz
    if (ret != 0) {
        LOG_ERR("Failed to set CTRL1_XL register (err: %d)", ret);
        return ret;
    }
    ret = lsm6dso_i2c_reg_write_byte(i2c_dev, LSM6DSO_REG_CTRL2_G, 0x40); // 104Hz
    if (ret != 0) {
        LOG_ERR("Failed to set CTRL2_G register (err: %d)", ret);
        return ret;
    }
    LOG_INF("LSM6DSO initialized successfully at %d Hz.", SAMPLING_RATE_HZ);
    return 0;
}
static int lsm6dso_fetch_accel_data(const struct device *i2c_dev, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t accel_data[6];
    int ret = lsm6dso_i2c_reg_read_bytes(i2c_dev, LSM6DSO_REG_OUTX_L_XL, accel_data, 6);
    if (ret != 0) {
        LOG_ERR("Failed to read accelerometer data (err: %d)", ret);
        return ret;
    }
    *ax = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    *ay = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    *az = (int16_t)(accel_data[4] | (accel_data[5] << 8));
    return 0;
}
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features_buffer + offset, length * sizeof(float));
    return 0;
}

// --- Main function ---
int main(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c30));
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("Edge Impulse live inferencing with LSM6DSO sensor (Zephyr)\n");

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device %s is not ready!", i2c_dev->name);
        return 0;
    }
    LOG_INF("I2C device %s is ready.", i2c_dev->name);

    if (lsm6dso_init(i2c_dev) != 0) {
        LOG_ERR("Failed to initialize LSM6DSO sensor.");
        return 0;
    }
    
    if (sizeof(features_buffer) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printf("ERROR: The size of the features_buffer is not correct. Expected %d items, but had %u\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (unsigned int)(sizeof(features_buffer) / sizeof(float)));
        return 1;
    }

    printf("Starting inferencing in 2-second cycles...\n");
    
    while (1) {
        // ==================================================================
        // MODIFICATION START: Simplified 2-second cycle loop
        // ==================================================================

        // --- Phase 1: Collect 2 seconds of data ---
        printf("\nCollecting %d samples for the next inference...\n", NUM_SAMPLES_PER_INFERENCE);

        for (uint32_t i = 0; i < NUM_SAMPLES_PER_INFERENCE; i++) {
            int64_t start_time_ms = k_uptime_get();

            int16_t ax_raw, ay_raw, az_raw;
            if (lsm6dso_fetch_accel_data(i2c_dev, &ax_raw, &ay_raw, &az_raw) == 0) {
                size_t current_index = i * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
                features_buffer[current_index + 0] = ax_raw * LSM6DSO_SENSITIVITY_G;
                features_buffer[current_index + 1] = ay_raw * LSM6DSO_SENSITIVITY_G;
                features_buffer[current_index + 2] = az_raw * LSM6DSO_SENSITIVITY_G;
                
                // Print progress dots
                if ((i + 1) % 10 == 0) {
                    printf(".");
                }
            } else {
                LOG_WRN("Failed to read sensor data, retrying sample %u...", i);
                i--; // Decrement counter to retry fetching this sample
            }

            // Wait until the next sample time
            int64_t processing_time_ms = k_uptime_get() - start_time_ms;
            int sleep_time_ms = SAMPLING_PERIOD_MS - processing_time_ms;
            if (sleep_time_ms > 0) {
                k_sleep(K_MSEC(sleep_time_ms));
            }
        }
        printf("\nData collection complete. Running inference...\n");

        // --- Phase 2: Run inference on the collected data ---
        ei_impulse_result_t result = { 0 };
        signal_t features_signal;
        features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        features_signal.get_data = &raw_feature_get_data;

        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
        if (res != 0) {
            printf("ERROR: run_classifier returned: %d\n", res);
        } else {
            printf("Predictions (DSP: %d ms, Classification: %d ms, Anomaly: %d ms):\n",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly);
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                printf("    %s: %.5f\n", result.classification[ix].label,
                                        result.classification[ix].value);
            }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
            printf("    anomaly score: %.3f\n", result.anomaly);
#endif
        }
        
        // Loop will now immediately start the next data collection cycle
        // ==================================================================
        // MODIFICATION END
        // ==================================================================
    }
    return 0;
}