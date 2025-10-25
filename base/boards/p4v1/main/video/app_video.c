#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "driver/ppa.h"
#include "driver/jpeg_encode.h"
#include "display.h" // Assumed external header for display functions
#include "app_video.h"
#include "overlay_manager.h"

#define OPTFLOW_WIDTH   70
#define OPTFLOW_HEIGHT  70

static const char *TAG = "app_video_unified";

/* --- Static Context Structures --- */

/**
 * @brief Frame operation callback function type
 */
typedef void (*app_video_frame_operation_cb_t)(uint8_t *camera_buf, 
                                              uint8_t camera_buf_index, 
                                              uint32_t camera_buf_hes, 
                                              uint32_t camera_buf_ves, 
                                              size_t camera_buf_len);

/**
 * @brief Video application context structure (from app_video.c)
 */
typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    struct v4l2_buffer v4l2_buf;
    uint8_t camera_mem_mode;
    app_video_frame_operation_cb_t user_frame_operation_cb;
    frame_update_t frame_update;
    TaskHandle_t video_stream_task_handle; // Unused in this file, kept for context
    bool video_task_delete;
    SemaphoreHandle_t video_stop_sem;
} app_video_context_t;

static app_video_context_t s_video_ctx;

/**
 * @brief Camera state flags (from app_video_stream.c)
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
 * @brief Camera state structure (from app_video_stream.c)
 */
typedef struct {
    camera_flags_t flags;
    uint32_t init_count;
    uint16_t current_interval_minutes;
    uint32_t next_wake_time; // Unused in this file, kept for context
} camera_state_t;

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

/**
 * @brief Camera buffer management structure (from app_video_stream.c)
 */
typedef struct {
    void *canvas_buf[EXAMPLE_CAM_BUF_NUM];
    uint8_t *scaled_camera_buf;
    uint8_t *shared_photo_buf;
    uint8_t *jpg_buf;
    uint32_t jpg_size; // Unused in this file, kept for context
    size_t rx_buffer_size;
    int video_cam_fd;
} camera_buffer_t;

static camera_buffer_t s_camera_buffer = {
    .video_cam_fd = -1
};

/* --- Static Utility Variables --- */

// PPA and JPEG handles (from app_video_utils.c)
static ppa_client_handle_t s_ppa_handle = NULL;
static jpeg_encoder_handle_t s_jpeg_handle = NULL;

// Scale configuration (from app_video_utils.c)
static const int s_scale_level_res[SCALE_LEVELS] = {960, 480, 240, 120};
// static const uint32_t s_adj_resolution_width[SCALE_LEVELS] = {1920, 960, 480, 240};
// static const uint32_t s_adj_resolution_height[SCALE_LEVELS] = {1080, 540, 270, 135};

// Cache and Rotation (from app_video_stream.c)
static size_t s_data_cache_line_size = 0;
static ppa_srm_rotation_angle_t s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;

// Optical flow (from app_video_stream.c)
static uint8_t g_frame[OPTFLOW_HEIGHT*OPTFLOW_WIDTH] = {0};
// Note: task_hangle_1 is not used but kept in context
// Note: The timer logic uses a local type 'timer_callback_t' which is simplified here.

/* --- Private Function Declarations (Internal Helpers) --- */

static esp_err_t video_stream_start(int video_fd);
static esp_err_t video_stream_stop(int video_fd);
static esp_err_t video_receive_frame(int video_fd);
static esp_err_t video_free_frame(int video_fd);
static void video_process_frame(int video_fd);
static bool validate_scale_level(int scale_level);

static esp_err_t initialize_buffers(void);
static void cleanup_buffers(void);
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len);
                                        
static uint8_t rgb565_to_grayscale(uint16_t rgb565);
static void resize_frame_nearest(const uint16_t* src_buffer, uint32_t width, uint32_t height, 
    uint8_t* dst_buffer, uint32_t new_width, uint32_t new_height);

/* -------------------------------------------------------------------------- */
/* Unified Function Implementations                      */
/* -------------------------------------------------------------------------- */


/* --- app_video_utils Implementations --- */

esp_err_t app_video_utils_init(void) {
    esp_err_t ret;
    
    // Initialize PPA (Pixel Processing Accelerator)
    ppa_client_config_t ppa_config = {
        .oper_type = PPA_OPERATION_SRM,
    };
    
    ret = ppa_register_client(&ppa_config, &s_ppa_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PPA client: 0x%x", ret);
        return ret;
    }

    // Initialize JPEG encoder
    jpeg_encode_engine_cfg_t jpeg_config = {
        .timeout_ms = 70,
    };

    ret = jpeg_new_encoder_engine(&jpeg_config, &s_jpeg_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create JPEG encoder: 0x%x", ret);
        ppa_unregister_client(s_ppa_handle);
        s_ppa_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "Video utilities initialized successfully");
    return ESP_OK;
}

esp_err_t app_video_utils_deinit(void) {
    // Clean up PPA
    if (s_ppa_handle != NULL) {
        ppa_unregister_client(s_ppa_handle);
        s_ppa_handle = NULL;
    }
    
    // Clean up JPEG encoder
    if (s_jpeg_handle != NULL) {
        jpeg_del_encoder_engine(s_jpeg_handle);
        s_jpeg_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Video utilities deinitialized");
    return ESP_OK;
}

static esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, 
    size_t out_buf_size, ppa_srm_rotation_angle_t rotation_angle) {
    // Validate input parameters
    if (in_buf == NULL || out_buf == NULL || in_width == 0 || in_height == 0 || out_width == 0 || out_height == 0) {
        ESP_LOGE(TAG, "Invalid buffer pointers or dimensions");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (crop_width > in_width || crop_height > in_height) {
        ESP_LOGE(TAG, "Crop region larger than input image");
        return ESP_ERR_INVALID_ARG;
    }

    // Configure PPA scale-rotate-mirror operation
    ppa_srm_oper_config_t srm_config = {
        .in = {
            .buffer = in_buf,
            .pic_w = in_width,
            .pic_h = in_height,
            .block_w = crop_width,
            .block_h = crop_height,
            .block_offset_x = (in_width - crop_width) / 2,
            .block_offset_y = (in_height - crop_height) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = out_buf,
            .buffer_size = out_buf_size,
            .pic_w = out_width,
            .pic_h = out_height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = rotation_angle,
        .scale_x = (float)out_width / crop_width,
        .scale_y = (float)out_height / crop_height,
        .rgb_swap = 0, // Assuming camera outputs correct RGB565 order
        .byte_swap = 0, // Assuming camera outputs correct byte order
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(s_ppa_handle, &srm_config);
}

esp_err_t app_image_process_video_frame(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    int scale_level, ppa_srm_rotation_angle_t rotation_angle,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size) {
    // Validate parameters
    if (!validate_scale_level(scale_level)) {
        ESP_LOGE(TAG, "Invalid scale level: %d", scale_level);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (in_buf == NULL || out_buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int res_width = s_scale_level_res[scale_level - 1];
    int res_height = s_scale_level_res[scale_level - 1];

    return app_image_process_scale_crop(
        in_buf, in_width, in_height,
        res_width, res_height,
        out_buf, out_width, out_height, out_buf_size,
        rotation_angle
    );
}

esp_err_t app_image_encode_jpeg(
    uint8_t *src_buf, uint32_t width, uint32_t height, uint8_t quality,
    uint8_t *out_buf, size_t out_buf_size, uint32_t *out_size) {
    // Validate parameters
    if (src_buf == NULL || out_buf == NULL || out_size == NULL || width == 0 || height == 0) {
        ESP_LOGE(TAG, "Invalid buffer pointers or dimensions");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (quality > 100) {
        ESP_LOGW(TAG, "JPEG quality clamped to 100");
        quality = 100;
    }

    // Configure JPEG encoding
    jpeg_encode_cfg_t enc_config = {
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = quality,
        .width = width,
        .height = height,
    };

    // Perform JPEG encoding
    esp_err_t ret = jpeg_encoder_process(
        s_jpeg_handle, 
        &enc_config, 
        src_buf, 
        width * height * 2,  // RGB565: 2 bytes per pixel
        out_buf, 
        out_buf_size, 
        out_size
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encoding failed: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "JPEG encoded: %" PRIu32 "x%" PRIu32 " -> %" PRIu32 " bytes", 
             width, height, *out_size);
    return ESP_OK;
}

void swap_rgb565_bytes(uint16_t *buffer, int pixel_count) {
    if (buffer == NULL || pixel_count <= 0) {
        return;
    }

    for (int i = 0; i < pixel_count; i++) {
        uint16_t pixel = buffer[i];
        buffer[i] = (pixel >> 8) | (pixel << 8);
    }
}

/* --- Private Utility Implementations --- */

static bool validate_scale_level(int scale_level) {
    return (scale_level >= 1 && scale_level <= SCALE_LEVELS);
}

/* --- app_video Implementations --- */

esp_err_t app_video_main(void) {
    const esp_video_init_csi_config_t base_csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port      = I2C_NUM_0,
                .scl_pin   = GPIO_NUM_13,
                .sda_pin   = GPIO_NUM_14,
            },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin  = -1,
    };

    esp_video_init_csi_config_t csi_config = base_csi_config;

    esp_video_init_config_t cam_config = {
        .csi = &csi_config,
    };

    return esp_video_init(&cam_config);
}

int app_video_open(char *dev, video_fmt_t init_fmt) {
    struct v4l2_format format;
    struct v4l2_capability capability;
    const int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Open video device
    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device: %s", dev);
        return -1;
    }

    // Query device capabilities
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
        ESP_LOGE(TAG, "Failed to get device capabilities");
        goto cleanup;
    }

    ESP_LOGI(TAG, "Driver: %s, Card: %s, Bus: %s", 
             capability.driver, capability.card, capability.bus_info);

    // Get current format
    memset(&format, 0, sizeof(format));
    format.type = buffer_type;
    if (ioctl(fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "Failed to get current format");
        goto cleanup;
    }

    ESP_LOGI(TAG, "Current resolution: %" PRIu32 "x%" PRIu32, 
             format.fmt.pix.width, format.fmt.pix.height);

    // Store resolution
    s_video_ctx.camera_buf_hes = format.fmt.pix.width;
    s_video_ctx.camera_buf_ves = format.fmt.pix.height;

    // Set desired format if different from current
    if (format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format new_format = {
            .type = buffer_type,
            .fmt.pix.width = format.fmt.pix.width,
            .fmt.pix.height = format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &new_format) != 0) {
            ESP_LOGE(TAG, "Failed to set pixel format");
            goto cleanup;
        }
        ESP_LOGI(TAG, "Set pixel format to: 0x%08X", init_fmt);
    }

    // Create synchronization semaphore
    s_video_ctx.video_stop_sem = xSemaphoreCreateBinary();
    if (s_video_ctx.video_stop_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create stop semaphore");
        goto cleanup;
    }

    return fd;

cleanup:
    close(fd);
    return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb) {
    // Validate buffer count
    if (fb_num > MAX_BUFFER_COUNT || fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %" PRIu32 " outside valid range (%d-%d)", fb_num, MIN_BUFFER_COUNT, MAX_BUFFER_COUNT);
        return ESP_ERR_INVALID_SIZE;
    }

    struct v4l2_requestbuffers req = {
        .count = fb_num,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP,
    };

    s_video_ctx.camera_mem_mode = req.memory;

    // Request buffers from driver
    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "Failed to request buffers");
        return ESP_FAIL;
    }

    // Query and map buffers
    int i;
    for (i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = req.memory,
            .index = i,
        };

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to query buffer %d", i);
            goto cleanup;
        }

        // Handle memory mapping or user pointers
        if (req.memory == V4L2_MEMORY_MMAP) {
            s_video_ctx.camera_buffer[i] = mmap(NULL, buf.length, 
                                              PROT_READ | PROT_WRITE, 
                                              MAP_SHARED, video_fd, 
                                              buf.m.offset);
            if (s_video_ctx.camera_buffer[i] == NULL) {
                ESP_LOGE(TAG, "Failed to mmap buffer %d", i);
                goto cleanup;
            }
        } else {
            if (fb == NULL || fb[i] == NULL) {
                ESP_LOGE(TAG, "User buffer %d is NULL", i);
                goto cleanup;
            }
            buf.m.userptr = (unsigned long)fb[i];
            s_video_ctx.camera_buffer[i] = (uint8_t *)fb[i];
        }

        s_video_ctx.camera_buf_size = buf.length;

        // Queue buffer for capture
        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "Failed to queue buffer %d", i);
            goto cleanup;
        }
    }

    ESP_LOGI(TAG, "Allocated %" PRIu32 " buffers of %zu bytes", fb_num, s_video_ctx.camera_buf_size);
    return ESP_OK;

cleanup:
    // Clean up already allocated buffers
    for (int j = 0; j < i; j++) {
        if (req.memory == V4L2_MEMORY_MMAP && s_video_ctx.camera_buffer[j] != NULL) {
            munmap(s_video_ctx.camera_buffer[j], s_video_ctx.camera_buf_size);
        }
        s_video_ctx.camera_buffer[j] = NULL;
    }
    return ESP_FAIL;
}

esp_err_t app_video_get_bufs(int fb_num, void **fb) {
    // Validate buffer count
    if (fb_num > MAX_BUFFER_COUNT || fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %d outside valid range (%d-%d)", fb_num, MIN_BUFFER_COUNT, MAX_BUFFER_COUNT);
        return ESP_ERR_INVALID_SIZE;
    }

    // Copy buffer pointers
    for (int i = 0; i < fb_num; i++) {
        if (s_video_ctx.camera_buffer[i] == NULL) {
            ESP_LOGE(TAG, "Buffer %d is NULL", i);
            return ESP_FAIL;
        }
        fb[i] = s_video_ctx.camera_buffer[i];
    }

    return ESP_OK;
}

uint32_t app_video_get_buf_size(void) {
    uint32_t bytes_per_pixel = (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565) ? 2 : 3;
    return s_video_ctx.camera_buf_hes * s_video_ctx.camera_buf_ves * bytes_per_pixel;
}

static esp_err_t video_stream_start(int video_fd) {
    ESP_LOGI(TAG, "Starting video stream");

    int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &buffer_type) != 0) {
        ESP_LOGE(TAG, "Failed to start video stream");
        return ESP_FAIL;
    }

    // Verify stream format
    struct v4l2_format format = {0};
    format.type = buffer_type;
    if (ioctl(video_fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "Failed to get stream format");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Video stream started: %" PRIu32 "x%" PRIu32, 
             format.fmt.pix.width, format.fmt.pix.height);
    return ESP_OK;
}

esp_err_t app_video_stream_task_start(int video_fd) {
    // Start video stream
    esp_err_t ret = video_stream_start(video_fd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream");
        return ret;
    }
    
    // Note: The original code does not explicitly create the task here, 
    // it assumes an external task calls app_video_capture().
    // We only perform the V4L2 streaming start.

    return ESP_OK;
}

esp_err_t app_video_stream_task_stop(int video_fd) {
    s_video_ctx.video_task_delete = true;
    ESP_LOGI(TAG, "Video stream task stop requested");
    return ESP_OK;
}

esp_err_t app_video_wait_video_stop(void) {
    if (s_video_ctx.video_stop_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_video_ctx.video_stop_sem, portMAX_DELAY) ? ESP_OK : ESP_FAIL;
}

static esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb) {
    if (operation_cb == NULL) {
        ESP_LOGE(TAG, "Frame operation callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_video_ctx.user_frame_operation_cb = operation_cb;
    ESP_LOGI(TAG, "Frame operation callback registered");
    return ESP_OK;
}

static esp_err_t app_video_register_frame_update(frame_update_t frame_update) {
    if (frame_update == NULL) {
        ESP_LOGE(TAG, "Frame operation callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_video_ctx.frame_update = frame_update;
    ESP_LOGI(TAG, "Frame operation callback registered");
    return ESP_OK;
}

static void app_video_update_frame(uint8_t *frame, uint32_t width, uint32_t height) {
    s_video_ctx.frame_update(frame, width, height);
}

esp_err_t app_video_close(int video_fd) {
    if (video_fd >= 0) {
        close(video_fd);
    }
    
    // Clean up semaphore
    if (s_video_ctx.video_stop_sem != NULL) {
        vSemaphoreDelete(s_video_ctx.video_stop_sem);
        s_video_ctx.video_stop_sem = NULL;
    }
    
    return ESP_OK;
}

static esp_err_t video_stream_stop(int video_fd) {
    ESP_LOGI(TAG, "Stopping video stream");

    int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &buffer_type) != 0) {
        ESP_LOGE(TAG, "Failed to stop video stream");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Video stream stopped");
    return ESP_OK;
}

static esp_err_t video_receive_frame(int video_fd) {
    memset(&s_video_ctx.v4l2_buf, 0, sizeof(s_video_ctx.v4l2_buf));
    s_video_ctx.v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_video_ctx.v4l2_buf.memory = s_video_ctx.camera_mem_mode;

    if (ioctl(video_fd, VIDIOC_DQBUF, &s_video_ctx.v4l2_buf) != 0) {
        // Handle common non-fatal errors like EAGAIN (if non-blocking) or EINTR
        if (errno != EAGAIN && errno != EINTR) {
             ESP_LOGE(TAG, "Failed to dequeue video frame: %s", strerror(errno));
        }
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t video_free_frame(int video_fd) {
    if (ioctl(video_fd, VIDIOC_QBUF, &s_video_ctx.v4l2_buf) != 0) {
        ESP_LOGE(TAG, "Failed to requeue video frame");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void video_process_frame(int video_fd) {
    // Update user pointer for userptr mode
    if (s_video_ctx.camera_mem_mode == V4L2_MEMORY_USERPTR) {
        s_video_ctx.v4l2_buf.m.userptr = (unsigned long)s_video_ctx.camera_buffer[s_video_ctx.v4l2_buf.index];
        s_video_ctx.v4l2_buf.length = s_video_ctx.camera_buf_size;
    }

    uint8_t buf_index = s_video_ctx.v4l2_buf.index;

    // Call user frame processing callback
    if (s_video_ctx.user_frame_operation_cb != NULL) {
        s_video_ctx.user_frame_operation_cb(
            s_video_ctx.camera_buffer[buf_index],
            buf_index,
            s_video_ctx.camera_buf_hes,
            s_video_ctx.camera_buf_ves,
            s_video_ctx.camera_buf_size
        );
    }
}

void app_video_capture(int video_fd) {
    // Receive frame from driver
    if (video_receive_frame(video_fd) != ESP_OK) {
        return;
    }

    // Process frame
    video_process_frame(video_fd);

    // Return frame to driver
    if (video_free_frame(video_fd) != ESP_OK) {
        ESP_LOGI(TAG, "Video frame free failed");
    }

    // Check for task termination request
    if (s_video_ctx.video_task_delete) {
        ESP_LOGI(TAG, "Video stream task stopping");
        s_video_ctx.video_task_delete = false;
        
        // Stop video stream and signal completion
        video_stream_stop(video_fd);
        if (s_video_ctx.video_stop_sem != NULL) {
            xSemaphoreGive(s_video_ctx.video_stop_sem);
        }
    }
}

/* --- app_video_stream Implementations --- */

static esp_err_t initialize_buffers(void) {
    esp_err_t ret = ESP_OK;
    
    // Get cache alignment for memory allocation (Moved from app_video_stream_init)
    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_data_cache_line_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get cache alignment: 0x%x", ret);
        return ret;
    }

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
    
    // Note: Use PHOTO_HEIGHT_1088P for 16-byte alignment needed by JPEG encoder
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
    
    // Free JPEG buffer (assumes it was allocated with a heap_caps_aligned_calloc fallback or is safe to free this way)
    if (s_camera_buffer.jpg_buf != NULL) {
        // In a proper JPEG context, this should be jpeg_free_encoder_mem(). We use heap_caps_free as a generic cleanup for now.
        heap_caps_free(s_camera_buffer.jpg_buf); 
        s_camera_buffer.jpg_buf = NULL;
    }
}

static uint8_t rgb565_to_grayscale(uint16_t rgb565) {
    // Extract RGB components from RGB565
    uint8_t r = (rgb565 >> 11) & 0x1F;  // 5 bits red
    uint8_t g = (rgb565 >> 5) & 0x3F;   // 6 bits green  
    uint8_t b = rgb565 & 0x1F;          // 5 bits blue
    
    // Convert to 8-bit values
    r = (r * 255) / 31;
    g = (g * 255) / 63;
    b = (b * 255) / 31;
    
    // Calculate grayscale using luminance formula
    return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

static void resize_frame_nearest(const uint16_t* src_buffer, uint32_t width, uint32_t height, 
    uint8_t* dst_buffer, uint32_t new_width, uint32_t new_height) {
    if (!src_buffer || !dst_buffer || width == 0 || height == 0 || new_width == 0 || new_height == 0) return;
    
    float x_ratio = (float)width / new_width;
    float y_ratio = (float)height / new_height;
    
    for (int y = 0; y < new_height; y++) {
        for (int x = 0; x < new_width; x++) {
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);
            
            src_x = (src_x < (int)width) ? src_x : (int)width - 1;
            src_y = (src_y < (int)height) ? src_y : (int)height - 1;
            
            uint16_t src_pixel = src_buffer[src_y * width + src_x];
            dst_buffer[y * new_width + x] = rgb565_to_grayscale(src_pixel);
        }
    }
}

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

    // Resize and convert for Optical Flow calculation
    resize_frame_nearest(
        (const uint16_t*)s_camera_buffer.canvas_buf[camera_buf_index], 
        BSP_LCD_H_RES, BSP_LCD_V_RES,
        g_frame, 
        OPTFLOW_WIDTH, OPTFLOW_HEIGHT);

    app_video_update_frame(g_frame, OPTFLOW_WIDTH, OPTFLOW_HEIGHT);
    
    // Convert RGB565 format for display
    swap_rgb565_bytes(s_camera_buffer.canvas_buf[camera_buf_index], BSP_LCD_H_RES * BSP_LCD_V_RES);    
    
    // Update display with processed frame
    update_display(s_camera_buffer.canvas_buf[camera_buf_index]);
}

esp_err_t app_video_stream_init(frame_update_t frame_update) {
    ESP_LOGI(TAG, "Initializing video streaming");

    // 1. Initialize video utilities (PPA/JPEG)
    esp_err_t ret = app_video_utils_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize video utils: 0x%x", ret);
        return ret;
    }
    
    // 2. Initialize main video system (CSI/I2C)
    ret = app_video_main();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Video main init failed: 0x%x", ret);
        goto cleanup_utils;
    }

    // 3. Open video device
    s_camera_buffer.video_cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    if (s_camera_buffer.video_cam_fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device");
        ret = ESP_FAIL;
        goto cleanup_utils; // No explicit app_video_main cleanup exists
    }

    // 4. Allocate all required buffers
    ret = initialize_buffers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buffers");
        goto cleanup_device;
    }

    // 5. Configure video buffers (MMAP mode: NULL for fb)
    ret = app_video_set_bufs(s_camera_buffer.video_cam_fd, EXAMPLE_CAM_BUF_NUM, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set video buffers: 0x%x", ret);
        goto cleanup_buffers;
    }

    // 6. Register frame processing callback
    ret = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame operation callback: 0x%x", ret);
        goto cleanup_buffers;
    }

    // 6. Register frame callback
    ret = app_video_register_frame_update(frame_update);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame operation callback: 0x%x", ret);
        goto cleanup_buffers;
    }

    // 7. Start video streaming (V4L2 STREAMON)
    ret = app_video_stream_task_start(s_camera_buffer.video_cam_fd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream task: 0x%x", ret);
        goto cleanup_buffers;
    }

    ESP_LOGI(TAG, "Video streaming initialized successfully");
    return ESP_OK;

cleanup_buffers:
    cleanup_buffers();
cleanup_device:
    app_video_close(s_camera_buffer.video_cam_fd);
    s_camera_buffer.video_cam_fd = -1;
cleanup_utils:
    app_video_utils_deinit();
    
    return ret;
}

esp_err_t app_video_stream_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing video stream");
    
    // Stop V4L2 streaming if still running (app_video_capture takes care of STREAMOFF on task_delete flag)
    if (s_camera_buffer.video_cam_fd >= 0) {
        app_video_stream_task_stop(s_camera_buffer.video_cam_fd);
        // Wait for the capture task (if running) to finish and call stream_stop and close
        // The task is external, so we just wait for the semaphore release
        app_video_wait_video_stop(); 
    }
    
    // Reset rotation state
    s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;
    
    // Clean up buffers
    cleanup_buffers();
    
    // Close video device (includes semaphore cleanup)
    if (s_camera_buffer.video_cam_fd >= 0) {
        app_video_close(s_camera_buffer.video_cam_fd);
        s_camera_buffer.video_cam_fd = -1;
    }
    
    // Deinitialize video utilities
    app_video_utils_deinit();
    
    // Reset camera state
    memset(&s_camera_state, 0, sizeof(s_camera_state));
    
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
    if (s_data_cache_line_size > 0) {
        *size = ALIGN_UP(PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1080P * 2, s_data_cache_line_size);
    } else {
        *size = PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1080P * 2;
    }
}

void app_video_stream_get_jpg_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.jpg_buf;
    *size = s_camera_buffer.rx_buffer_size;
}

void app_video_stream_get_shared_photo_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.shared_photo_buf;
    *size = SHARED_PHOTO_BUF_WIDTH * SHARED_PHOTO_BUF_HEIGHT * 2;
}

// Note: app_video_stream_capture() is now just a wrapper for app_video_capture()
void app_video_stream_capture(void) {
    app_video_capture(s_camera_buffer.video_cam_fd);
}
