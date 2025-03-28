#include <M5Unified.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        2       // G2
#define I2C_MASTER_SCL_IO        1       // G1
#define SEN54_ADDR               0x69    // SEL=GND → 0x69

// SEN54 commands
#define START_MEASUREMENT        0x0021
#define READ_MEASUREMENT         0x03C4
#define START_FAN_CLEANING       0x5607
#define DEVICE_RESET             0xD304

static const char *TAG = "SEN54";

typedef struct {
    float pm1;
    float pm25;
    float pm4;
    float pm10;
    float tvoc;
    float temperature;
    float humidity;
} SensorReadings;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

esp_err_t sen54_write_cmd(uint16_t command) {
    uint8_t cmd[2] = {(uint8_t)(command >> 8), (uint8_t)(command & 0xFF)};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, SEN54_ADDR, cmd, sizeof(cmd), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write command 0x%04X failed: 0x%x", command, ret);
    }
    return ret;
}

esp_err_t sen54_read_data(uint8_t *data, size_t len) {
    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, SEN54_ADDR, data, len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: 0x%x", ret);
    }
    return ret;
}

void initialize_sensor() {
    ESP_LOGI(TAG, "Resetting sensor");
    ESP_ERROR_CHECK(sen54_write_cmd(DEVICE_RESET));
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Starting fan cleaning");
    ESP_ERROR_CHECK(sen54_write_cmd(START_FAN_CLEANING));
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    ESP_LOGI(TAG, "Starting measurements");
    ESP_ERROR_CHECK(sen54_write_cmd(START_MEASUREMENT));
    vTaskDelay(pdMS_TO_TICKS(2000));
}

bool read_sensor_values(SensorReadings &readings) {
    uint8_t data[24] = {0};
    
    // Send read command
    if(sen54_write_cmd(READ_MEASUREMENT) != ESP_OK) {
        return false;
    }
    
    // Add small delay before reading
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Read data
    if(sen54_read_data(data, sizeof(data)) != ESP_OK) {
        return false;
    }
    
    // Parse values (SEN54 data format)
    readings.pm1 = ((data[0] << 8) | data[1]) / 10.0f;
    readings.pm25 = ((data[3] << 8) | data[4]) / 10.0f;
    readings.pm4 = ((data[6] << 8) | data[7]) / 10.0f;
    readings.pm10 = ((data[9] << 8) | data[10]) / 10.0f;
    readings.humidity = ((data[12] << 8) | data[13]) / 100.0f;
    readings.temperature = ((data[15] << 8) | data[16]) / 200.0f;
    readings.tvoc = ((data[18] << 8) | data[19]) / 10.0f;
    
    // Validate readings
    if (readings.pm1 > 1000 || readings.pm25 > 1000 || 
        readings.temperature > 100 || readings.temperature < -40 ||
        readings.humidity > 100 || readings.humidity < 0) {
        ESP_LOGE(TAG, "Invalid sensor readings");
        return false;
    }
    
    return true;
}

void update_display(const SensorReadings &readings) {
    M5.Display.startWrite();
    M5.Display.clear(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(2);
    
    int y = 10;
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("PM1.0: %.1f μg/m³", readings.pm1);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("PM2.5: %.1f μg/m³", readings.pm25);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("PM4.0: %.1f μg/m³", readings.pm4);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("PM10:  %.1f μg/m³", readings.pm10);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("TVOC:  %.1f ppb", readings.tvoc);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("Temp:  %.1f°C", readings.temperature);
    M5.Display.setCursor(10, y); y += 20;
    M5.Display.printf("Hum:   %.1f%%", readings.humidity);
    
    M5.Display.endWrite();
}

void check_i2c_bus() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SEN54_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor found at address 0x%02X", SEN54_ADDR);
    } else {
        ESP_LOGE(TAG, "Sensor not found at address 0x%02X (error: 0x%x)", SEN54_ADDR, ret);
    }
}

extern "C" void app_main() {
    M5.begin();
    M5.Display.setRotation(3);
    M5.Display.clear(TFT_BLACK);

    ESP_LOGI(TAG, "Initializing I2C");
    i2c_master_init();
    
    check_i2c_bus();
    
    ESP_LOGI(TAG, "Initializing SEN54");
    initialize_sensor();

    SensorReadings readings = {0};
    
    while (true) {
        if (read_sensor_values(readings)) {
            update_display(readings);
            
            ESP_LOGI(TAG, "Readings: PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f TVOC=%.1f Temp=%.1f°C Hum=%.1f%%",
                    readings.pm1, readings.pm25, readings.pm4, readings.pm10,
                    readings.tvoc, readings.temperature, readings.humidity);
        } else {
            ESP_LOGW(TAG, "Failed to read sensor - retrying...");
            M5.Display.clear(TFT_RED);
            M5.Display.setCursor(10, 10);
            M5.Display.print("Sensor Error!");
            
            // Try to reinitialize sensor
            initialize_sensor();
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}