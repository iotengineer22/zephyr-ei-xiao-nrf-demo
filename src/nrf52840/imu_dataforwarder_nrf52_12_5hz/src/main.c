#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>


LOG_MODULE_REGISTER(lsm6dso_i2c_example, LOG_LEVEL_INF);

// --- Constant definitions ---
// CHANGED: Sampling rate updated from 104Hz to 12.5Hz
#define SAMPLING_RATE_HZ 12.5f
// Corresponding period for 12.5Hz (in milliseconds)
#define SAMPLING_PERIOD_MS (int)(1000.0f / SAMPLING_RATE_HZ) // Should be 80 ms

// Sensitivity for the accelerometer at a Full Scale of ±2g. Unit: g/LSB
#define LSM6DSO_SENSITIVITY_G (0.000061f)

// --- LSM6DSO I2C address and register definitions ---
#define LSM6DSO_I2C_ADDR    0x6A // LSM6DSO I2C device address

#define LSM6DSO_REG_WHO_AM_I 0x0F // Identification register
#define LSM6DSO_WHO_AM_I_VAL 0x6A // Expected WHO_AM_I value

#define LSM6DSO_REG_CTRL1_XL 0x10 // Accelerometer control register
#define LSM6DSO_REG_CTRL2_G  0x11 // Gyroscope control register
// Accelerometer/gyroscope data output registers (low byte first)
#define LSM6DSO_REG_OUTX_L_XL 0x28 // Accelerometer X axis low byte
#define LSM6DSO_REG_OUTX_L_G  0x22 // Gyroscope X axis low byte

// --- Data structure definitions ---
// Structure for storing raw sensor data
struct lsm6dso_raw_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

// --- Helper functions ---

/**
 * @brief Write a single byte to an LSM6DSO register via I2C.
 */
static int lsm6dso_i2c_reg_write_byte(const struct device *i2c_dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t tx_buf[2] = {reg_addr, value};
    return i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), LSM6DSO_I2C_ADDR);
}

/**
 * @brief Read a single byte from an LSM6DSO register via I2C.
 */
static int lsm6dso_i2c_reg_read_byte(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *value)
{
    return i2c_reg_read_byte(i2c_dev, LSM6DSO_I2C_ADDR, reg_addr, value);
}

/**
 * @brief Read multiple consecutive bytes from LSM6DSO register via I2C.
 */
static int lsm6dso_i2c_reg_read_bytes(const struct device *i2c_dev, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    return i2c_burst_read(i2c_dev, LSM6DSO_I2C_ADDR, reg_addr, data, len);
}

// --- LSM6DSO driver core functionality ---

/**
 * @brief Initialize the LSM6DSO sensor.
 * Check WHO_AM_I and set ODR for accelerometer and gyroscope.
 */
static int lsm6dso_init(const struct device *i2c_dev)
{
    uint8_t who_am_i = 0;
    int ret;

    // Verify device ID
    ret = lsm6dso_i2c_reg_read_byte(i2c_dev, LSM6DSO_REG_WHO_AM_I, &who_am_i);
    if (ret != 0) {
        LOG_ERR("Failed to read WHO_AM_I register (err: %d)", ret);
        return ret;
    }
    if (who_am_i != LSM6DSO_WHO_AM_I_VAL) {
        LOG_ERR("Invalid WHO_AM_I: 0x%02x, expected 0x%02x", who_am_i, LSM6DSO_WHO_AM_I_VAL);
        return -ENODEV;
    }
    LOG_INF("LSM6DSO WHO_AM_I check passed. ID: 0x%02x", who_am_i);

    // --- CHANGED: Set sensor's Output Data Rate (ODR) to 12.5Hz ---
    // From datasheet, 12.5Hz ODR is '0001', ±2g range is '00'.
    // CTRL1_XL register value: 0b00010000 = 0x10
    ret = lsm6dso_i2c_reg_write_byte(i2c_dev, LSM6DSO_REG_CTRL1_XL, 0x10);
    if (ret != 0) {
        LOG_ERR("Failed to set CTRL1_XL register (err: %d)", ret);
        return ret;
    }

    // From datasheet, 12.5Hz ODR is '0001', ±250dps range is '00'.
    // CTRL2_G register value: 0b00010000 = 0x10
    ret = lsm6dso_i2c_reg_write_byte(i2c_dev, LSM6DSO_REG_CTRL2_G, 0x10);
    if (ret != 0) {
        LOG_ERR("Failed to set CTRL2_G register (err: %d)", ret);
        return ret;
    }

    LOG_INF("LSM6DSO initialized successfully at 12.5 Hz.");
    return 0;
}

/**
 * @brief Fetch raw accelerometer and gyroscope data from LSM6DSO sensor.
 * @param i2c_dev Pointer to I2C device structure.
 * @param raw_data_out Pointer to structure for storing raw data.
 * @return 0 on success, negative value on failure.
 */
static int lsm6dso_fetch_raw_data(const struct device *i2c_dev, struct lsm6dso_raw_data *raw_data_out)
{
    uint8_t accel_data[6];
    // uint8_t gyro_data[6]; // Gyroscope reading is commented out, so this is not needed.
    int ret;

    // Read accelerometer data (6 bytes)
    ret = lsm6dso_i2c_reg_read_bytes(i2c_dev, LSM6DSO_REG_OUTX_L_XL, accel_data, 6);
    if (ret != 0) {
        // LOG_ERR is resource-intensive; commenting it out is recommended in a high-speed loop.
        // LOG_ERR("Failed to read accelerometer data (err: %d).", ret);
        return ret;
    }
    raw_data_out->accel_x = (int16_t)(accel_data[0] | (accel_data[1] << 8));
    raw_data_out->accel_y = (int16_t)(accel_data[2] | (accel_data[3] << 8));
    raw_data_out->accel_z = (int16_t)(accel_data[4] | (accel_data[5] << 8));

    // // Read gyroscope data (6 bytes)
    // ret = lsm6dso_i2c_reg_read_bytes(i2c_dev, LSM6DSO_REG_OUTX_L_G, gyro_data, 6);
    // if (ret != 0) {
    //     // LOG_ERR("Failed to read gyroscope data (err: %d).", ret);
    //     return ret;
    // }
    // raw_data_out->gyro_x = (int16_t)(gyro_data[0] | (gyro_data[1] << 8));
    // raw_data_out->gyro_y = (int16_t)(gyro_data[2] | (gyro_data[3] << 8));
    // raw_data_out->gyro_z = (int16_t)(gyro_data[4] | (gyro_data[5] << 8));

    return 0;
}

/**
 * @brief Display raw accelerometer and gyroscope data.
 * @param raw_data Pointer to structure containing raw data.
 */
static void lsm6dso_display_raw_data(const struct lsm6dso_raw_data *raw_data)
{
    // Print each value right-aligned in a 6-digit field, separated by a tab, and end with a newline.
    // printf("A: %6d, %6d, %6d\t G: %6d, %6d, %6d\r\n",
    //        raw_data->accel_x, raw_data->accel_y, raw_data->accel_z,
    //        raw_data->gyro_x, raw_data->gyro_y, raw_data->gyro_z);

    // printf("%6d %6d %6d\t\n",
    //        raw_data->accel_x, raw_data->accel_y, raw_data->accel_z);

	// Convert raw values to 'g' by multiplying with sensitivity.
    float ax_g = raw_data->accel_x * LSM6DSO_SENSITIVITY_G;
    float ay_g = raw_data->accel_y * LSM6DSO_SENSITIVITY_G;
    float az_g = raw_data->accel_z * LSM6DSO_SENSITIVITY_G;

    // Print each value separated by a tab, with 4 decimal places.
    printf("% 8.4f\t% 8.4f\t% 8.4f\r\n", ax_g, ay_g, az_g);
}

// --- Main function ---

int main(void)
{
    // const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c30));
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    struct lsm6dso_raw_data sensor_data;

    // Disable buffering for stdout to ensure immediate output for printf.
    setvbuf(stdout, NULL, _IONBF, 0);

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device %s is not ready!", i2c_dev->name);
        return 0;
    }
    LOG_INF("I2C device %s is ready.", i2c_dev->name);

    if (lsm6dso_init(i2c_dev) != 0) {
        LOG_ERR("Failed to initialize LSM6DSO sensor.");
        return 0;
    }

    // CHANGED: Format specifier updated to print float value for SAMPLING_RATE_HZ
    printf("LSM6DSO polling at %.1f Hz. Raw Data Output:\n\n", SAMPLING_RATE_HZ);

    // --- Precise timing control for the main loop ---
    int64_t start_time_ms;
    int64_t processing_time_ms;
    int sleep_time_ms;

    while (1) {
        // Record the start time of the loop.
        start_time_ms = k_uptime_get();

        // Fetch and display the data.
        if (lsm6dso_fetch_raw_data(i2c_dev, &sensor_data) == 0) {
            lsm6dso_display_raw_data(&sensor_data);
        }

        // Calculate the time spent on fetching and displaying data.
        processing_time_ms = k_uptime_get() - start_time_ms;

        // Calculate the remaining time to wait until the next sampling moment.
        sleep_time_ms = SAMPLING_PERIOD_MS - processing_time_ms;

        // If there is remaining time, sleep for that duration.
        // This ensures the total loop period approaches SAMPLING_PERIOD_MS.
        if (sleep_time_ms > 0) {
            k_sleep(K_MSEC(sleep_time_ms));
        }
    }

    return 0;
}