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
#include "app_video.h"
#include "optflow.h"
#include "overlay_manager.h"

// Optical Flow Constants
#define OPTFLOW_WIDTH   50
#define OPTFLOW_HEIGHT  70

// Camera Configuration
#define BSP_MIPI_CAMERA_XCLK_FREQUENCY  (24000000)  // 24MHz
#define BSP_CAMERA_EN_PIN     (GPIO_NUM_12)
#define BSP_CAMERA_RST_PIN    (GPIO_NUM_26)
#define BSP_CAMERA_XCLK_PIN   (GPIO_NUM_11)

typedef void (*timer_callback_t)(void*);

static const char *TAG = "main";

TaskHandle_t task_hangle_1 = NULL;
TaskHandle_t task_hangle_2 = NULL;

static uint8_t g_frame[OPTFLOW_HEIGHT * OPTFLOW_WIDTH] = {0};
static int g_dx = 0;
static int g_dy = 0;
static char g_frame_captured = 0;

void platform_toggle_led(char led) {

}

void platform_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t platform_time_ms(void) {
    return esp_timer_get_time();
}

static void create_timer(timer_callback_t callback, uint64_t freq) {
  const esp_timer_create_args_t timer_args = {
    .callback = callback,
    .name = "Timer"
  };
  esp_timer_handle_t timer_handler;
  esp_timer_create(&timer_args, &timer_handler);
  esp_timer_start_periodic(timer_handler, 1000000/freq);
}

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

static void frame_update(uint16_t *buffer, uint32_t width, uint32_t height) {
    // Resize and Grayscale for Optical Flow Input
    resize_frame_nearest(buffer, width, height,
        g_frame, OPTFLOW_WIDTH, OPTFLOW_HEIGHT);

    g_frame_captured = 1;

    // Byte Swap for Display
    swap_rgb565_bytes(buffer, BSP_LCD_H_RES * BSP_LCD_V_RES);

    // Draw an arrow
    draw_arrow(buffer, width, height, 
               width/2, height/2, 
               width/2 + g_dx, height/2 + g_dy,
               COLOR_BLUE, 5);
    
    // Update Display
    update_display(buffer);
}

static void calc_optflow(void*) {
    if (g_frame_captured == 0) return;
    g_frame_captured = 0;

    static uint64_t t0 = 0;
    uint64_t t1 = platform_time_ms();
    int dt = t1 - t0;
    t0 = t1;
    int fps = (double)1000000 / dt;
    float clearity = 0;
    float dx = 0;
    float dy = 0;
    optflow_calc(g_frame, &dx, &dy, &clearity);
    ESP_LOGI(TAG, "dx: %d\t\tdy: %d\t\tclear: %d\tFPS: %d", (int)(dx*1000), (int)(dy*1000), (int)(clearity*1000), fps);
    g_dx = (int)(dx * 100);
    g_dy = (int)(dy * 100);
}

static void capture_video(void*) {
    if (g_frame_captured == 0) {
        app_video_stream_capture();
    }
}

void core0() {
    // Initialize optical flow
    optflow_init(OPTFLOW_WIDTH, OPTFLOW_HEIGHT);

    // Initialize display
    init_display();
    
    // Initialize camera XCLK
    ESP_LOGI(TAG, "Initializing camera XCLK");
    ESP_ERROR_CHECK(bsp_camera_xclk_init());
    
    // Initialize video streaming
    ESP_LOGI(TAG, "Initializing video streaming");
    ESP_ERROR_CHECK(app_video_stream_init(frame_update));

    create_timer(capture_video, 15);
    create_timer(calc_optflow, 100);
    while (true) { 
        // capture_video(NULL);
        // calc_optflow(NULL);
        platform_delay(1000); 
    }
}

void core1() {
    while (1) { 
        platform_delay(1000); 
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Start program");

    xTaskCreatePinnedToCore(core0, "Core 0", 4096, NULL, 1, &task_hangle_1, 0);
    xTaskCreatePinnedToCore(core1, "Core 1", 4096, NULL, 1, &task_hangle_2, 1);

    while (1) { 
        platform_delay(1000); 
    }
}

// Auto-initialize camera power on startup
static void __attribute__((constructor)) bsp_p4_eye_auto_init(void) {
    bsp_camera_power_init();
}