#include <stdio.h>
#include <esp_log.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_cam_sensor_xclk.h>
#include <esp_err.h>
#include <platform.h>
#include "display.h"
#include "app_video_stream.h"

// Camera Configuration
#define BSP_MIPI_CAMERA_XCLK_FREQUENCY  (24000000)  // 24MHz
#define BSP_CAMERA_EN_PIN     (GPIO_NUM_12)
#define BSP_CAMERA_RST_PIN    (GPIO_NUM_26)
#define BSP_CAMERA_XCLK_PIN   (GPIO_NUM_11)

static const char *TAG = "main";

static void bsp_camera_power_init(void) {
    // Initialize camera power control
    rtc_gpio_init(BSP_CAMERA_EN_PIN);
    rtc_gpio_set_direction(BSP_CAMERA_EN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(BSP_CAMERA_EN_PIN);
    rtc_gpio_pullup_dis(BSP_CAMERA_EN_PIN);
    rtc_gpio_hold_dis(BSP_CAMERA_EN_PIN);
    rtc_gpio_set_level(BSP_CAMERA_EN_PIN, 1);
    rtc_gpio_hold_en(BSP_CAMERA_EN_PIN);
    
    // Initialize camera reset pin
    gpio_set_level(BSP_CAMERA_RST_PIN, 1);
}

static esp_err_t bsp_camera_xclk_init(void) {
    static esp_cam_sensor_xclk_handle_t xclk_handle = NULL;

    esp_cam_sensor_xclk_config_t cam_xclk_config = {
        .esp_clock_router_cfg = {
            .xclk_pin = BSP_CAMERA_XCLK_PIN,
            .xclk_freq_hz = BSP_MIPI_CAMERA_XCLK_FREQUENCY,
        }
    };
    
    esp_err_t ret = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate XCLK: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_cam_sensor_xclk_start(xclk_handle, &cam_xclk_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start XCLK: %s", esp_err_to_name(ret));
        xclk_handle = NULL;
    }
    
    return ret;
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting ESP32-P4-EYE application");

    // Initialize display
    ESP_LOGI(TAG, "Initializing display");
    init_display();
    
    // Initialize camera XCLK
    ESP_LOGI(TAG, "Initializing camera XCLK");
    ESP_ERROR_CHECK(bsp_camera_xclk_init());
    
    // Initialize I2C bus
    ESP_LOGI(TAG, "Initializing I2C bus");
    // ESP_ERROR_CHECK(bsp_i2c_init());
    
    // Initialize video streaming
    ESP_LOGI(TAG, "Initializing video streaming");
    ESP_ERROR_CHECK(app_video_stream_init());
    
    ESP_LOGI(TAG, "Application initialization completed successfully");
    
    // Main loop can be added here for ongoing tasks
    while (true) {
        app_video_stream_capture();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Auto-initialize camera power on startup
static void __attribute__((constructor)) bsp_p4_eye_auto_init(void) {
    bsp_camera_power_init();
}