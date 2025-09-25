#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "uros_network_interfaces.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include "esp_timer.h"

#define BNO055_I2C_ADDR 0x28
#define BNO055_CHIP_ID_REG 0x00
#define BNO055_CHIP_ID_VAL 0xA0
#define BNO055_SYS_TRIGGER_REG 0x3F
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_CONFIG_MODE 0x00
#define BNO055_NDOF_MODE 0x0C
#define BNO055_ACC_X_LSB_REG 0x08
#define BNO055_ACC_RANGE 2
#define BNO055_GYR_X_LSB_REG 0x14
#define BNO055_GYRO_RANGE 2000
#define BNO055_QUA_W_LSB_REG 0x20
#define BNO055_QUA_X_LSB_REG 0x22
#define BNO055_QUA_Y_LSB_REG 0x24
#define BNO055_QUA_Z_LSB_REG 0x26
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_TIMEOUT_MS 1000

static const char *TAG = "BNO055";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static SemaphoreHandle_t sensor_mutex;

// Sensor data structure to avoid global variables
typedef struct
{
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t quat_w, quat_x, quat_y, quat_z;
} bno055_data_t;

static bno055_data_t sensor_data;

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

// Helper: Read 16-bit signed value from register
esp_err_t bno055_read_int16(uint8_t reg, int16_t *value)
{
    uint8_t data[2];
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), I2C_TIMEOUT_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        *value = 0;
        return ret;
    }
    *value = (int16_t)((data[1] << 8) | data[0]);
    return ESP_OK;
}

// Initialize I2C master
esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
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

    vTaskDelay(pdMS_TO_TICKS(50));                                   // Wait for boot
    esp_err_t ret = bno055_write_byte(BNO055_SYS_TRIGGER_REG, 0x20); // Soft reset
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(650)); // POR time

    uint8_t chip_id;
    if (bno055_read_byte(BNO055_CHIP_ID_REG, &chip_id) != ESP_OK || chip_id != BNO055_CHIP_ID_VAL)
    {
        ESP_LOGE(TAG, "Chip ID read failed: 0x%02X (expected 0x%02X)", chip_id, BNO055_CHIP_ID_VAL);
        return false;
    }
    ESP_LOGI(TAG, "BNO055 detected: Chip ID 0x%02X", chip_id);

    bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE);
    vTaskDelay(pdMS_TO_TICKS(20));
    bno055_write_byte(BNO055_OPR_MODE_REG, BNO055_NDOF_MODE);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "BNO055 initialized in NDOF mode");
    return true;
}

// BNO055 task
void bno055_task(void *pvParameter)
{
    if (!bno055_init())
    {
        ESP_LOGE(TAG, "Init failed, restarting in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    while (1)
    {
        bno055_data_t data;
        bool read_success = true;

        // Read sensor data
        read_success &= (bno055_read_int16(BNO055_ACC_X_LSB_REG, &data.acc_x) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_ACC_X_LSB_REG + 2, &data.acc_y) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_ACC_X_LSB_REG + 4, &data.acc_z) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_GYR_X_LSB_REG, &data.gyro_x) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_GYR_X_LSB_REG + 2, &data.gyro_y) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_GYR_X_LSB_REG + 4, &data.gyro_z) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_QUA_W_LSB_REG, &data.quat_w) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_QUA_X_LSB_REG, &data.quat_x) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_QUA_Y_LSB_REG, &data.quat_y) == ESP_OK);
        read_success &= (bno055_read_int16(BNO055_QUA_Z_LSB_REG, &data.quat_z) == ESP_OK);

        if (read_success)
        {
            if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                sensor_data = data;
                xSemaphoreGive(sensor_mutex);
            }
            ESP_LOGI(TAG, "Accel (mg): X=%d Y=%d Z=%d | Gyro (mdps): X=%d Y=%d Z=%d | Quat: W=%d X=%d Y=%d Z=%d",
                     data.acc_x, data.acc_y, data.acc_z, data.gyro_x, data.gyro_y, data.gyro_z,
                     data.quat_w, data.quat_x, data.quat_y, data.quat_z);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

#define RCCHECK(fn)                                                                           \
    {                                                                                         \
        rcl_ret_t temp_rc = fn;                                                               \
        if (temp_rc != RCL_RET_OK)                                                            \
        {                                                                                     \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                                \
        }                                                                                     \
    }
#define RCSOFTCHECK(fn)                                                                         \
    {                                                                                           \
        rcl_ret_t temp_rc = fn;                                                                 \
        if (temp_rc != RCL_RET_OK)                                                              \
        {                                                                                       \
            ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc); \
        }                                                                                       \
    }

static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu imu_msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL)
        return;

    bno055_data_t data;
    bool data_valid = false;

    // Read sensor data with mutex protection
    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        data = sensor_data;
        data_valid = true;
        xSemaphoreGive(sensor_mutex);
    }

    if (data_valid)
    {
        // Convert acceleration to m/s² (1 mg = 0.00981 m/s²)
        imu_msg.linear_acceleration.x = (float)data.acc_x * 0.00981;
        imu_msg.linear_acceleration.y = (float)data.acc_y * 0.00981;
        imu_msg.linear_acceleration.z = (float)data.acc_z * 0.00981;

        // Convert gyroscope to rad/s (1 mdps = 0.0000174533 rad/s)
        imu_msg.angular_velocity.x = (float)data.gyro_x * 16.0 * 0.0000174533;
        imu_msg.angular_velocity.y = (float)data.gyro_y * 16.0 * 0.0000174533;
        imu_msg.angular_velocity.z = (float)data.gyro_z * 16.0 * 0.0000174533;

        // Convert quaternion to normalized values
        float scale = 1.0 / (1 << 14); // 2^-14
        imu_msg.orientation.w = (float)data.quat_w * scale;
        imu_msg.orientation.x = (float)data.quat_x * scale;
        imu_msg.orientation.y = (float)data.quat_y * scale;
        imu_msg.orientation.z = (float)data.quat_z * scale;

        rcutils_time_point_value_t now;
        if (rmw_uros_epoch_nanos(&now) == RMW_RET_OK)
        {
            imu_msg.header.stamp.sec = now / 1000000000LL;
            imu_msg.header.stamp.nanosec = now % 1000000000LL;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to get timestamp, using system time");
            int64_t ticks = esp_timer_get_time();
            imu_msg.header.stamp.sec = ticks / 1000000;
            imu_msg.header.stamp.nanosec = (ticks % 1000000) * 1000;
        }
        // Set header
        imu_msg.header.frame_id.data = "imu_link";
        imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
        imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

        // Set covariances (example values, adjust based on sensor specs)
        imu_msg.orientation_covariance[0] = 0.01;         // Example: variance for orientation
        imu_msg.angular_velocity_covariance[0] = 0.0001;  // Example: variance for gyro
        imu_msg.linear_acceleration_covariance[0] = 0.01; // Example: variance for accel
        for (int i = 1; i < 9; i++)
        {
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = 0.0;
        }

        RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));

        ESP_LOGI(TAG, "Published IMU data: Accel X=%.2f m/s² Y=%.2f m/s² Z=%.2f m/s², "
                      "Gyro X=%.2f rad/s Y=%.2f rad/s Z=%.2f rad/s, "
                      "Quat W=%.2f X=%.2f Y=%.2f Z=%.2f",
                 imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                 imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                 imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire sensor data for publishing");
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "IMU_Sensor", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"));

    rcl_timer_t timer;
    const unsigned int timer_timeout = 100; // 10 Hz
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
    sensor_mutex = xSemaphoreCreateMutex();
    if (sensor_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }



    xTaskCreate(bno055_task, "bno055_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "uros_task", 4096, NULL, 5, NULL);
}