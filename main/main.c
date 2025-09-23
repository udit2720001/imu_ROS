#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h" // New I2C driver API (v5.x+)
#include "esp_log.h"
#include "esp_err.h"

#include <string.h>

#include <unistd.h>

#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h" // New oneshot API

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// BNO055 definitions (from datasheet, Page 0 registers)
#define BNO055_I2C_ADDR 0x28 // 7-bit address
#define BNO055_CHIP_ID_REG 0x00
#define BNO055_CHIP_ID_VAL 0xA0 // Expected chip ID
#define BNO055_SYS_TRIGGER_REG 0x3F
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_CONFIG_MODE 0x00 // Config mode
#define BNO055_NDOF_MODE 0x0C   // 9-axis fusion mode
#define BNO055_ACC_X_LSB_REG 0x08
#define BNO055_ACC_RANGE 2 // Default ±2g (1 LSB = 1 mg)
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 1000

static const char *TAG = "BNO055";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

// Helper: Write byte to register
esp_err_t bno055_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

// Helper: Read single byte from register
esp_err_t bno055_read_byte(uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read byte failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Helper: Read 16-bit signed value from register (little-endian: LSB first)
int16_t bno055_read_int16(uint8_t reg)
{
    uint8_t data[2];
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return 0;
    }
    return (int16_t)((data[1] << 8) | data[0]); // MSB << 8 | LSB
}

// Initialize I2C master (new v5+ API)
esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1, // Auto-select port
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO055_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Initialize BNO055
bool bno055_init(void)
{
    if (i2c_init() != ESP_OK)
        return false;

    // Wait for boot (~19ms after POR)
    vTaskDelay(pdMS_TO_TICKS(50));

    // Soft reset (bit 7 of SYS_TRIGGER)
    esp_err_t ret = bno055_write_byte(BNO055_SYS_TRIGGER_REG, 0x20);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(650)); // POR time ~650ms max

    // Verify chip ID
    uint8_t chip_id;
    if (bno055_read_byte(BNO055_CHIP_ID_REG, &chip_id) != ESP_OK || chip_id != BNO055_CHIP_ID_VAL)
    {
        ESP_LOGE(TAG, "Chip ID read failed: 0x%02X (expected 0x%02X)", chip_id, BNO055_CHIP_ID_VAL);
        return false;
    }
    ESP_LOGI(TAG, "BNO055 detected: Chip ID 0x%02X", chip_id);

    // Set to CONFIG mode
    bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Set to NDOF mode (fusion enabled)
    bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_NDOF_MODE);
    vTaskDelay(pdMS_TO_TICKS(20)); // Stabilize

    ESP_LOGI(TAG, "BNO055 initialized in NDOF mode");
    return true;
}

// Task to read and print accelerometer data
void bno055_task(void *pvParameter)
{
    if (!bno055_init())
    {
        ESP_LOGE(TAG, "Init failed, restarting...");
        esp_restart();
    }

    while (1)
    {
        int16_t acc_x = bno055_read_int16(BNO055_ACC_X_LSB_REG);
        int16_t acc_y = bno055_read_int16(BNO055_ACC_X_LSB_REG + 2);
        int16_t acc_z = bno055_read_int16(BNO055_ACC_X_LSB_REG + 4);

        // Scale to mg (for ±2g: 1 LSB = 1 mg)
        float ax = (float)acc_x;
        float ay = (float)acc_y;
        float az = (float)acc_z;

        ESP_LOGI(TAG, "Accel (mg): X=%.2fg Y=%.2fg Z=%.2fg", ax / 100.0, ay / 100.0, az / 100.0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(bno055_task, "bno055_task", 4096, NULL, 5, NULL);
}