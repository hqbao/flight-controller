#include <sys/stat.h> 
#include <dirent.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_private/esp_cache_private.h>
#include <driver/ppa.h>
#include <driver/jpeg_encode.h>
#include <esp_timer.h>
#include "app_video.h"
#include "app_video_stream.h"
#include "app_video_utils.h"
#include "display.h"
#include "overlay_manager.h"
#include "optflow.h"

#define OPTFLOW_WIDTH   70
#define OPTFLOW_HEIGHT  70

static const char *TAG = "app_video_stream";

/* Type definitions */

/**
 * @brief Camera state flags
 */
typedef struct {
    unsigned int is_initialized : 1;
    unsigned int is_take_photo : 1;
    unsigned int is_take_video : 1;
    unsigned int is_interval_photo_active : 1;
    unsigned int is_flash_light_on : 1;
    unsigned int reserved : 27;
} camera_flags_t;

/**
 * @brief Camera state structure
 */
typedef struct {
    camera_flags_t flags;
    uint32_t init_count;
    uint16_t current_interval_minutes;
    uint32_t next_wake_time;
} camera_state_t;

/**
 * @brief Camera buffer management structure
 */
typedef struct {
    void *canvas_buf[EXAMPLE_CAM_BUF_NUM];
    uint8_t *scaled_camera_buf;
    uint8_t *shared_photo_buf;
    uint8_t *jpg_buf;
    uint32_t jpg_size;
    size_t rx_buffer_size;
    int video_cam_fd;
} camera_buffer_t;

typedef void (*timer_callback_t)(void*);

/* Static variables */
static camera_state_t s_camera_state = {
    .flags.is_initialized = false,
    .flags.is_take_photo = false,
    .flags.is_take_video = false,
    .flags.is_interval_photo_active = false,
    .flags.is_flash_light_on = false,
    .init_count = 0,
    .current_interval_minutes = 0,
    .next_wake_time = 0,
};

static camera_buffer_t s_camera_buffer = {
    .video_cam_fd = -1
};

static size_t s_data_cache_line_size = 0;
static ppa_srm_rotation_angle_t s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;

static uint8_t g_frame[OPTFLOW_HEIGHT*OPTFLOW_WIDTH] = {0};
TaskHandle_t task_hangle_1 = NULL;
static char g_frame_captured = 0;
static int g_dx = 0;
static int g_dy = 0;

/* Private function declarations */
static esp_err_t initialize_buffers(void);
static void cleanup_buffers(void);
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len);

static void create_timer(timer_callback_t callback, uint64_t freq) {
  const esp_timer_create_args_t timer_args = {
    .callback = callback,
    .name = "Timer"
  };
  esp_timer_handle_t timer_handler;
  esp_timer_create(&timer_args, &timer_handler);
  esp_timer_start_periodic(timer_handler, 1000000/freq);
}

static void calc_otpflw(void*) {
    if (g_frame_captured == 0) return;
    g_frame_captured = 0;

    static uint64_t t0 = 0;
    uint64_t t1 = esp_timer_get_time();
    int dt = t1 - t0;
    t0 = t1;
    int fps = (double)1000000 / dt;
    float clearity = 0;
    float dx = 0;
    float dy = 0;
    optflow_calc(g_frame, &dx, &dy, &clearity);
    ESP_LOGI(TAG, "dx: %d\t\tdy: %d\t\tclear: %d\tFPS: %d", (int)(dx*1000), (int)(dy*1000), (int)(clearity*1000), fps);
    g_dx = dx * 100;
    g_dy = dy * 100;
}

/* Public function implementations */

esp_err_t app_video_stream_init() {
    optflow_init(OPTFLOW_WIDTH, OPTFLOW_HEIGHT);
    create_timer(calc_otpflw, 100);

    ESP_LOGI(TAG, "Initializing video streaming");

    // Initialize video utilities
    esp_err_t ret = app_video_utils_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize video utils: 0x%x", ret);
        return ret;
    }
    
    // Get cache alignment for memory allocation
    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_data_cache_line_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get cache alignment: 0x%x", ret);
        goto cleanup_utils;
    }

    // Initialize main video system
    ret = app_video_main();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Video main init failed: 0x%x", ret);
        goto cleanup_utils;
    }

    // Open video device
    s_camera_buffer.video_cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    if (s_camera_buffer.video_cam_fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device");
        ret = ESP_FAIL;
        goto cleanup_video;
    }

    // Allocate all required buffers
    ret = initialize_buffers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buffers");
        goto cleanup_device;
    }

    // Configure video buffers
    ret = app_video_set_bufs(s_camera_buffer.video_cam_fd, EXAMPLE_CAM_BUF_NUM, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set video buffers: 0x%x", ret);
        goto cleanup_buffers;
    }

    // Register frame processing callback
    ret = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame operation callback: 0x%x", ret);
        goto cleanup_buffers;
    }

    // Start video streaming task
    ret = app_video_stream_task_start(s_camera_buffer.video_cam_fd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream task: 0x%x", ret);
        goto cleanup_buffers;
    }

    ESP_LOGI(TAG, "Video streaming initialized successfully");
    return ESP_OK;

// Error handling (cleanup in reverse order)
cleanup_buffers:
    cleanup_buffers();
cleanup_device:
    app_video_close(s_camera_buffer.video_cam_fd);
    s_camera_buffer.video_cam_fd = -1;
cleanup_video:
    // Note: app_video_main cleanup would go here if available
cleanup_utils:
    app_video_utils_deinit();
    
    return ret;
}

esp_err_t app_video_stream_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing video stream");
    
    // Reset rotation state
    s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;
    
    // Clean up buffers
    cleanup_buffers();
    
    // Close video device
    if (s_camera_buffer.video_cam_fd >= 0) {
        app_video_close(s_camera_buffer.video_cam_fd);
        s_camera_buffer.video_cam_fd = -1;
    }
    
    // Deinitialize video utilities
    app_video_utils_deinit();
    
    // Reset camera state
    s_camera_state = (camera_state_t){
        .flags.is_initialized = false,
        .flags.is_take_photo = false,
        .flags.is_take_video = false,
        .flags.is_interval_photo_active = false,
        .flags.is_flash_light_on = false,
        .init_count = 0,
        .current_interval_minutes = 0,
        .next_wake_time = 0,
    };
    
    ESP_LOGI(TAG, "Video stream deinitialized successfully");
    return ESP_OK;
}

esp_err_t app_video_stream_set_flash_light(bool is_on) {
    s_camera_state.flags.is_flash_light_on = is_on;
    ESP_LOGI(TAG, "Flash light %s", is_on ? "ON" : "OFF");
    return ESP_OK;
}

bool app_video_stream_get_flash_light_state(void) {
    return s_camera_state.flags.is_flash_light_on;
}

bool app_video_stream_get_interval_photo_state(void) {
    return s_camera_state.flags.is_interval_photo_active;
}

uint16_t app_video_stream_get_current_interval_minutes(void) {
    return s_camera_state.current_interval_minutes;
}

int app_video_stream_get_video_fd(void) {
    return s_camera_buffer.video_cam_fd;
}

void app_video_stream_get_scaled_camera_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.scaled_camera_buf;
    *size = ALIGN_UP(PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1080P * 2, s_data_cache_line_size);
}

void app_video_stream_get_jpg_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.jpg_buf;
    *size = s_camera_buffer.rx_buffer_size;
}

void app_video_stream_get_shared_photo_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.shared_photo_buf;
    *size = SHARED_PHOTO_BUF_WIDTH * SHARED_PHOTO_BUF_HEIGHT * 2;
}

void app_video_stream_capture(void) {
    app_video_capture(s_camera_buffer.video_cam_fd);
}

/* Private function implementations */

/**
 * @brief Initialize all required buffers for video streaming
 */
static esp_err_t initialize_buffers(void) {
    esp_err_t ret = ESP_OK;
    
    // Allocate canvas buffers for display
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        size_t canvas_size = BSP_LCD_H_RES * BSP_LCD_V_RES * 2;
        s_camera_buffer.canvas_buf[i] = heap_caps_aligned_calloc(
            s_data_cache_line_size, 1, canvas_size, MALLOC_CAP_SPIRAM);
            
        if (s_camera_buffer.canvas_buf[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate canvas buffer %d (%zu bytes)", i, canvas_size);
            ret = ESP_ERR_NO_MEM;
            goto error;
        }
    }

    // Allocate scaled camera buffer for 1080P processing
    size_t scaled_buf_size = PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1080P * 2;
    s_camera_buffer.scaled_camera_buf = heap_caps_aligned_calloc(
        s_data_cache_line_size, 1, scaled_buf_size, MALLOC_CAP_SPIRAM);
        
    if (s_camera_buffer.scaled_camera_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate scaled camera buffer (%zu bytes)", scaled_buf_size);
        ret = ESP_ERR_NO_MEM;
        goto error;
    }

    // Allocate shared photo buffer (720P max)
    size_t shared_buf_size = SHARED_PHOTO_BUF_WIDTH * SHARED_PHOTO_BUF_HEIGHT * 2;
    s_camera_buffer.shared_photo_buf = heap_caps_aligned_calloc(
        s_data_cache_line_size, 1, shared_buf_size, MALLOC_CAP_SPIRAM);
        
    if (s_camera_buffer.shared_photo_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate shared photo buffer (%zu bytes)", shared_buf_size);
        ret = ESP_ERR_NO_MEM;
        goto error;
    }
    ESP_LOGI(TAG, "Allocated shared photo buffer: %zu bytes", shared_buf_size);

    // Allocate JPEG encoding buffer
    jpeg_encode_memory_alloc_cfg_t rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    
    size_t jpg_buf_size = (PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1088P * 2) / JPEG_COMPRESSION_RATIO;
    s_camera_buffer.jpg_buf = (uint8_t*)jpeg_alloc_encoder_mem(
        jpg_buf_size, &rx_mem_cfg, &s_camera_buffer.rx_buffer_size);
        
    if (s_camera_buffer.jpg_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer (%zu bytes)", jpg_buf_size);
        ret = ESP_ERR_NO_MEM;
        goto error;
    }

    return ESP_OK;

error:
    cleanup_buffers();
    return ret;
}

/**
 * @brief Clean up all allocated buffers
 */
static void cleanup_buffers(void) {
    // Free canvas buffers
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        if (s_camera_buffer.canvas_buf[i] != NULL) {
            heap_caps_free(s_camera_buffer.canvas_buf[i]);
            s_camera_buffer.canvas_buf[i] = NULL;
        }
    }
    
    // Free scaled camera buffer
    if (s_camera_buffer.scaled_camera_buf != NULL) {
        heap_caps_free(s_camera_buffer.scaled_camera_buf);
        s_camera_buffer.scaled_camera_buf = NULL;
    }
    
    // Free shared photo buffer
    if (s_camera_buffer.shared_photo_buf != NULL) {
        heap_caps_free(s_camera_buffer.shared_photo_buf);
        s_camera_buffer.shared_photo_buf = NULL;
    }
    
    // Free JPEG buffer (if allocated through jpeg_alloc_encoder_mem)
    if (s_camera_buffer.jpg_buf != NULL) {
        // Note: This might need jpeg_free_encoder_mem() depending on implementation
        heap_caps_free(s_camera_buffer.jpg_buf);
        s_camera_buffer.jpg_buf = NULL;
    }
}

// Function to convert RGB565 to grayscale (0-255)
static uint8_t rgb565_to_grayscale(uint16_t rgb565) {
    // Extract RGB components from RGB565
    uint8_t r = (rgb565 >> 11) & 0x1F;  // 5 bits red
    uint8_t g = (rgb565 >> 5) & 0x3F;   // 6 bits green  
    uint8_t b = rgb565 & 0x1F;          // 5 bits blue
    
    // Convert to 8-bit values (scale 5-bit to 8-bit: x * 255/31, 6-bit: x * 255/63)
    r = (r * 255) / 31;
    g = (g * 255) / 63;
    b = (b * 255) / 31;
    
    // Calculate grayscale using luminance formula
    return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

// Simple nearest-neighbor resize
static void resize_frame_nearest(const uint16_t* src_buffer, uint32_t width, uint32_t height, 
    uint8_t* dst_buffer, uint32_t new_width, uint32_t new_height) {
    float x_ratio = (float)width / new_width;
    float y_ratio = (float)height / new_height;
    
    for (int y = 0; y < new_height; y++) {
        for (int x = 0; x < new_width; x++) {
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);
            
            // Ensure we don't go out of bounds
            src_x = (src_x < width) ? src_x : width - 1;
            src_y = (src_y < height) ? src_y : height - 1;
            
            uint16_t src_pixel = src_buffer[src_y * width + src_x];
            dst_buffer[y * new_width + x] = rgb565_to_grayscale(src_pixel);
        }
    }
}

/**
 * @brief Process incoming camera video frames
 */
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len) {
    // Camera initialization sequence
    if (!s_camera_state.flags.is_initialized) {
        s_camera_state.init_count++;
        if (s_camera_state.init_count >= CAMERA_INIT_FRAMES) {
            s_camera_state.flags.is_initialized = true;
            s_camera_state.init_count = 0;
            ESP_LOGI(TAG, "Camera initialized after %d frames", CAMERA_INIT_FRAMES);
        }
    }

    // ESP_LOGI(TAG, "Size: %dx%d (%d)", camera_buf_hes, camera_buf_ves, camera_buf_len);
    // Process frame with current rotation
    esp_err_t ret = app_image_process_video_frame(
        camera_buf, camera_buf_hes, camera_buf_ves,
        1, s_current_ppa_rotation,
        s_camera_buffer.canvas_buf[camera_buf_index], BSP_LCD_H_RES, BSP_LCD_V_RES,
        ALIGN_UP(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_data_cache_line_size)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to process frame: 0x%x", ret);
        return;
    }

    resize_frame_nearest(
        s_camera_buffer.canvas_buf[camera_buf_index], 
        BSP_LCD_H_RES, BSP_LCD_V_RES,
        g_frame, 
        OPTFLOW_WIDTH, OPTFLOW_HEIGHT);
    g_frame_captured = 1;
    
    // // Convert RGB565 format for display
    // swap_rgb565_bytes(s_camera_buffer.canvas_buf[camera_buf_index], BSP_LCD_H_RES * BSP_LCD_V_RES);    
    
    // // Draw an arrow
    // uint16_t *frame_buffer = (uint16_t *)s_camera_buffer.canvas_buf[camera_buf_index];
    // draw_arrow(frame_buffer, BSP_LCD_V_RES, BSP_LCD_H_RES, 
    //            BSP_LCD_V_RES/2, BSP_LCD_H_RES/2, 
    //            BSP_LCD_V_RES/2 + g_dx, BSP_LCD_H_RES/2 + g_dy, 
    //            COLOR_BLUE, 5);

    // // Update display with processed frame
    // update_display(s_camera_buffer.canvas_buf[camera_buf_index]);
}
