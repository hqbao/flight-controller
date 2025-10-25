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
#include "display.h" 
#include "app_video.h"

static const char *TAG = "app_video_unified";

/* -------------------------------------------------------------------------- */
/* --- Static Context Structures --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Frame operation callback function type (Internal use)
 */
typedef void (*app_video_frame_operation_cb_t)(uint8_t *camera_buf, 
                                              uint8_t camera_buf_index, 
                                              uint32_t camera_buf_hes, 
                                              uint32_t camera_buf_ves, 
                                              size_t camera_buf_len);

/**
 * @brief Video application context, primarily for V4L2 buffer management.
 */
typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint32_t camera_buf_hes;        // Horizontal effective size (width)
    uint32_t camera_buf_ves;        // Vertical effective size (height)
    struct v4l2_buffer v4l2_buf;    // Current V4L2 buffer struct
    uint8_t camera_mem_mode;        // V4L2_MEMORY_MMAP or V4L2_MEMORY_USERPTR
    app_video_frame_operation_cb_t user_frame_operation_cb;
    frame_update_t frame_update;
    bool video_task_delete;         // Flag to request stream termination
    SemaphoreHandle_t video_stop_sem; // Semaphore to signal stream task completion
} app_video_context_t;

static app_video_context_t s_video_ctx = {
    .video_task_delete = false,
    .video_stop_sem = NULL,
};

/**
 * @brief Camera state flags (Reduced to essential flags).
 */
typedef struct {
    unsigned int is_initialized : 1;
    unsigned int is_flash_light_on : 1;
    unsigned int reserved : 30;
} camera_flags_t;

/**
 * @brief Camera state structure (Reduced).
 */
typedef struct {
    camera_flags_t flags;
    uint32_t init_count;
} camera_state_t;

static camera_state_t s_camera_state = {
    .flags = {0},
    .init_count = 0,
};

/**
 * @brief Camera buffer management structure (Reduced).
 */
typedef struct {
    void *canvas_buf[EXAMPLE_CAM_BUF_NUM]; // Buffers for display output
    uint8_t *jpg_buf;                       // Buffer for JPEG output
    uint32_t jpg_size;                      // Reserved for context
    size_t rx_buffer_size;                  // Actual size of allocated JPEG buffer
    int video_cam_fd;                       // V4L2 video device file descriptor
} camera_buffer_t;

static camera_buffer_t s_camera_buffer = {
    .video_cam_fd = -1
};

/* -------------------------------------------------------------------------- */
/* --- Static Utility Variables --- */
/* -------------------------------------------------------------------------- */

// Hardware Accelerator Handles
static ppa_client_handle_t s_ppa_handle = NULL;
static jpeg_encoder_handle_t s_jpeg_handle = NULL;

// Scaling configuration
static const int s_scale_level_res[SCALE_LEVELS] = {960, 480, 240, 120};

// Cache and Rotation State
static size_t s_data_cache_line_size = 0;
static ppa_srm_rotation_angle_t s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;

// Optical Flow Frame Buffer (Grayscale, 70x70)
static uint8_t g_frame[OPTFLOW_HEIGHT * OPTFLOW_WIDTH] = {0};

/* -------------------------------------------------------------------------- */
/* --- Private Function Declarations (Internal Helpers) --- */
/* -------------------------------------------------------------------------- */

// V4L2 Stream Management
static esp_err_t video_stream_start(int video_fd);
static esp_err_t video_stream_stop(int video_fd);
static esp_err_t video_receive_frame(int video_fd);
static esp_err_t video_free_frame(int video_fd);
static void video_process_frame(int video_fd);

// Buffer Management
static esp_err_t initialize_buffers(void);
static void cleanup_buffers(void);

// Image Processing
static esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, 
    size_t out_buf_size, ppa_srm_rotation_angle_t rotation_angle);
static esp_err_t app_image_process_video_frame(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    int scale_level, ppa_srm_rotation_angle_t rotation_angle,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size);
static bool validate_scale_level(int scale_level);
static void swap_rgb565_bytes(uint16_t *buffer, int pixel_count);
static uint8_t rgb565_to_grayscale(uint16_t rgb565);
static void resize_frame_nearest(const uint16_t* src_buffer, uint32_t width, uint32_t height, 
    uint8_t* dst_buffer, uint32_t new_width, uint32_t new_height);
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len);

// Context Callbacks
static esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb);
static esp_err_t app_video_register_frame_update(frame_update_t frame_update);
static void app_video_update_frame(uint8_t *frame, uint32_t width, uint32_t height);
static esp_err_t app_video_wait_video_stop(void);


/* -------------------------------------------------------------------------- */
/* --- Unified Implementations (V4L2) --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the main video system (CSI/I2C).
 */
static esp_err_t app_video_main(void) {
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

/**
 * @brief Open the video device and set initial format.
 */
static int app_video_open(char *dev, video_fmt_t init_fmt) {
    struct v4l2_format format;
    struct v4l2_capability capability;
    const int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device: %s", dev);
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
        ESP_LOGE(TAG, "Failed to get device capabilities");
        goto cleanup;
    }

    // Get and store current resolution
    memset(&format, 0, sizeof(format));
    format.type = buffer_type;
    if (ioctl(fd, VIDIOC_G_FMT, &format) != 0) {
        ESP_LOGE(TAG, "Failed to get current format");
        goto cleanup;
    }

    s_video_ctx.camera_buf_hes = format.fmt.pix.width;
    s_video_ctx.camera_buf_ves = format.fmt.pix.height;
    ESP_LOGI(TAG, "Current resolution: %" PRIu32 "x%" PRIu32, 
             s_video_ctx.camera_buf_hes, s_video_ctx.camera_buf_ves);

    // Set desired pixel format
    if (format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format new_format = {
            .type = buffer_type,
            .fmt.pix.width = s_video_ctx.camera_buf_hes,
            .fmt.pix.height = s_video_ctx.camera_buf_ves,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &new_format) != 0) {
            ESP_LOGE(TAG, "Failed to set pixel format 0x%08X", init_fmt);
            goto cleanup;
        }
        ESP_LOGI(TAG, "Set pixel format to: 0x%08X", init_fmt);
    }

    // Create stream stop semaphore
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

/**
 * @brief Configure and queue buffers (MMAP or USERPTR).
 */
static esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb) {
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

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "Failed to request buffers");
        return ESP_FAIL;
    }

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

        // Map buffer memory or assign user pointer
        if (req.memory == V4L2_MEMORY_MMAP) {
            s_video_ctx.camera_buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
            if (s_video_ctx.camera_buffer[i] == NULL) {
                ESP_LOGE(TAG, "Failed to mmap buffer %d", i);
                goto cleanup;
            }
        } else { // V4L2_MEMORY_USERPTR
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

    ESP_LOGI(TAG, "Allocated %" PRIu32 " buffers of %zu bytes in mode %s", 
             fb_num, s_video_ctx.camera_buf_size, req.memory == V4L2_MEMORY_MMAP ? "MMAP" : "USERPTR");
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

/**
 * @brief Start the video stream (VIDIOC_STREAMON).
 */
static esp_err_t video_stream_start(int video_fd) {
    int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &buffer_type) != 0) {
        ESP_LOGE(TAG, "Failed to start video stream");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Video stream started");
    return ESP_OK;
}

/**
 * @brief Stop the video stream (VIDIOC_STREAMOFF).
 */
static esp_err_t video_stream_stop(int video_fd) {
    int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &buffer_type) != 0) {
        ESP_LOGE(TAG, "Failed to stop video stream");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Video stream stopped");
    return ESP_OK;
}

/**
 * @brief Dequeue a video frame buffer (VIDIOC_DQBUF).
 */
static esp_err_t video_receive_frame(int video_fd) {
    memset(&s_video_ctx.v4l2_buf, 0, sizeof(s_video_ctx.v4l2_buf));
    s_video_ctx.v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    s_video_ctx.v4l2_buf.memory = s_video_ctx.camera_mem_mode;

    if (ioctl(video_fd, VIDIOC_DQBUF, &s_video_ctx.v4l2_buf) != 0) {
        if (errno != EAGAIN && errno != EINTR) {
             ESP_LOGE(TAG, "Failed to dequeue video frame: %s", strerror(errno));
        }
        return ESP_FAIL;
    }

    // Update user pointer for USERPTR mode (needed before processing)
    if (s_video_ctx.camera_mem_mode == V4L2_MEMORY_USERPTR) {
        s_video_ctx.v4l2_buf.m.userptr = (unsigned long)s_video_ctx.camera_buffer[s_video_ctx.v4l2_buf.index];
        s_video_ctx.v4l2_buf.length = s_video_ctx.camera_buf_size;
    }

    return ESP_OK;
}

/**
 * @brief Process the dequeued frame using the registered callback.
 */
static void video_process_frame(int video_fd) {
    uint8_t buf_index = s_video_ctx.v4l2_buf.index;

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

/**
 * @brief Requeue the processed buffer (VIDIOC_QBUF).
 */
static esp_err_t video_free_frame(int video_fd) {
    if (ioctl(video_fd, VIDIOC_QBUF, &s_video_ctx.v4l2_buf) != 0) {
        ESP_LOGE(TAG, "Failed to requeue video frame");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Close the video device and clean up related resources.
 */
static esp_err_t app_video_close(int video_fd) {
    if (video_fd >= 0) {
        close(video_fd);
    }
    
    if (s_video_ctx.video_stop_sem != NULL) {
        vSemaphoreDelete(s_video_ctx.video_stop_sem);
        s_video_ctx.video_stop_sem = NULL;
    }
    
    return ESP_OK;
}

/**
 * @brief Waits for the video stream to stop (signals on stop semaphore).
 */
static esp_err_t app_video_wait_video_stop(void) {
    if (s_video_ctx.video_stop_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    // Block indefinitely until the semaphore is given (stream task completion)
    return xSemaphoreTake(s_video_ctx.video_stop_sem, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

/* -------------------------------------------------------------------------- */
/* --- Unified Implementations (Image Processing Utilities) --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize PPA (Pixel Processing Accelerator) and JPEG Encoder.
 */
static esp_err_t app_video_utils_init(void) {
    esp_err_t ret;
    
    // Initialize PPA
    ppa_client_config_t ppa_config = {
        .oper_type = PPA_OPERATION_SRM, // Scale, Rotate, Mirror
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

    ESP_LOGI(TAG, "Video utilities initialized successfully (PPA, JPEG)");
    return ESP_OK;
}

/**
 * @brief Deinitialize PPA and JPEG Encoder.
 */
static esp_err_t app_video_utils_deinit(void) {
    if (s_ppa_handle != NULL) {
        ppa_unregister_client(s_ppa_handle);
        s_ppa_handle = NULL;
    }
    
    if (s_jpeg_handle != NULL) {
        jpeg_del_encoder_engine(s_jpeg_handle);
        s_jpeg_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Video utilities deinitialized");
    return ESP_OK;
}

/**
 * @brief Helper to perform PPA Scale, Rotate, and Crop.
 */
static esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, 
    size_t out_buf_size, ppa_srm_rotation_angle_t rotation_angle) {
    
    if (in_buf == NULL || out_buf == NULL || in_width == 0 || in_height == 0 || out_width == 0 || out_height == 0) {
        ESP_LOGE(TAG, "Invalid buffer pointers or dimensions");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (crop_width > in_width || crop_height > in_height) {
        ESP_LOGE(TAG, "Crop region larger than input image");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate center crop offsets
    uint32_t offset_x = (in_width - crop_width) / 2;
    uint32_t offset_y = (in_height - crop_height) / 2;

    ppa_srm_oper_config_t srm_config = {
        .in = {
            .buffer = in_buf,
            .pic_w = in_width,
            .pic_h = in_height,
            .block_w = crop_width,
            .block_h = crop_height,
            .block_offset_x = offset_x,
            .block_offset_y = offset_y,
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
        .rgb_swap = 0, 
        .byte_swap = 0, 
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(s_ppa_handle, &srm_config);
}

/**
 * @brief Process video frame with scaling based on pre-defined levels.
 */
static esp_err_t app_image_process_video_frame(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    int scale_level, ppa_srm_rotation_angle_t rotation_angle,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size) {
    
    if (!validate_scale_level(scale_level) || in_buf == NULL || out_buf == NULL) {
        ESP_LOGE(TAG, "Invalid scale level (%d) or buffers", scale_level);
        return ESP_ERR_INVALID_ARG;
    }

    int res_width = s_scale_level_res[scale_level - 1];
    int res_height = s_scale_level_res[scale_level - 1]; // Assume square crop

    return app_image_process_scale_crop(
        in_buf, in_width, in_height,
        res_width, res_height,
        out_buf, out_width, out_height, out_buf_size,
        rotation_angle
    );
}

/**
 * @brief Public function to encode a buffer to JPEG using the hardware encoder.
 */
esp_err_t app_image_encode_jpeg(
    uint8_t *src_buf, uint32_t width, uint32_t height, uint8_t quality,
    uint8_t *out_buf, size_t out_buf_size, uint32_t *out_size) {
    
    if (src_buf == NULL || out_buf == NULL || out_size == NULL || width == 0 || height == 0) {
        ESP_LOGE(TAG, "Invalid buffer pointers or dimensions");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (quality > 100) {
        quality = 100;
    }

    jpeg_encode_cfg_t enc_config = {
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = quality,
        .width = width,
        .height = height,
    };

    esp_err_t ret = jpeg_encoder_process(
        s_jpeg_handle, 
        &enc_config, 
        src_buf, 
        width * height * 2,  // RGB565 is 2 bytes per pixel
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

/**
 * @brief Swaps the high and low bytes of each 16-bit RGB565 pixel.
 */
static void swap_rgb565_bytes(uint16_t *buffer, int pixel_count) {
    if (buffer == NULL || pixel_count <= 0) {
        return;
    }

    for (int i = 0; i < pixel_count; i++) {
        uint16_t pixel = buffer[i];
        buffer[i] = (pixel >> 8) | (pixel << 8);
    }
}

/**
 * @brief Converts a single RGB565 pixel to 8-bit grayscale using a standard luminance formula.
 */
static uint8_t rgb565_to_grayscale(uint16_t rgb565) {
    // Extract 5-bit R, 6-bit G, 5-bit B
    uint8_t r_5 = (rgb565 >> 11) & 0x1F; 
    uint8_t g_6 = (rgb565 >> 5) & 0x3F;   
    uint8_t b_5 = rgb565 & 0x1F;          
    
    // Scale to 8-bit (0-255)
    uint8_t r_8 = (r_5 * 255) / 31;
    uint8_t g_8 = (g_6 * 255) / 63;
    uint8_t b_8 = (b_5 * 255) / 31;
    
    // Grayscale luminance: Y = 0.299*R + 0.587*G + 0.114*B
    return (uint8_t)(0.299f * r_8 + 0.587f * g_8 + 0.114f * b_8);
}

/**
 * @brief Resizes an RGB565 frame to a grayscale frame using nearest neighbor interpolation.
 */
static void resize_frame_nearest(const uint16_t* src_buffer, uint32_t width, uint32_t height, 
    uint8_t* dst_buffer, uint32_t new_width, uint32_t new_height) {
    
    if (!src_buffer || !dst_buffer || width == 0 || height == 0 || new_width == 0 || new_height == 0) return;
    
    float x_ratio = (float)width / new_width;
    float y_ratio = (float)height / new_height;
    
    for (int y = 0; y < new_height; y++) {
        for (int x = 0; x < new_width; x++) {
            // Calculate nearest source pixel coordinates
            int src_x = (int)(x * x_ratio);
            int src_y = (int)(y * y_ratio);
            
            // Boundary checks
            src_x = MIN(src_x, (int)width - 1);
            src_y = MIN(src_y, (int)height - 1);
            
            uint16_t src_pixel = src_buffer[src_y * width + src_x];
            dst_buffer[y * new_width + x] = rgb565_to_grayscale(src_pixel);
        }
    }
}

static bool validate_scale_level(int scale_level) {
    return (scale_level >= 1 && scale_level <= SCALE_LEVELS);
}

/* -------------------------------------------------------------------------- */
/* --- Unified Implementations (Buffer Management) --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Allocates all required internal buffers (canvas and jpeg).
 */
static esp_err_t initialize_buffers(void) {
    esp_err_t ret = ESP_OK;
    
    // Get cache alignment
    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_data_cache_line_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get cache alignment: 0x%x", ret);
        return ret;
    }

    // Allocate canvas buffers (display resolution, RGB565)
    size_t canvas_size = BSP_LCD_H_RES * BSP_LCD_V_RES * 2;
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        s_camera_buffer.canvas_buf[i] = heap_caps_aligned_calloc(
            s_data_cache_line_size, 1, canvas_size, MALLOC_CAP_SPIRAM);
            
        if (s_camera_buffer.canvas_buf[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate canvas buffer %d (%zu bytes)", i, canvas_size);
            ret = ESP_ERR_NO_MEM;
            goto error;
        }
    }

    // Allocate JPEG encoding buffer
    jpeg_encode_memory_alloc_cfg_t rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    
    // Buffer size based on compressed 1080P (using aligned height)
    size_t jpg_buf_size = (PHOTO_WIDTH_1080P * PHOTO_HEIGHT_1088P * 2) / JPEG_COMPRESSION_RATIO;
    s_camera_buffer.jpg_buf = (uint8_t*)jpeg_alloc_encoder_mem(
        jpg_buf_size, &rx_mem_cfg, &s_camera_buffer.rx_buffer_size);
        
    if (s_camera_buffer.jpg_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate JPEG buffer (%zu bytes)", jpg_buf_size);
        ret = ESP_ERR_NO_MEM;
        goto error;
    }
    ESP_LOGI(TAG, "Internal buffers allocated successfully");

    return ESP_OK;

error:
    cleanup_buffers();
    return ret;
}

/**
 * @brief Frees all internal buffers.
 */
static void cleanup_buffers(void) {
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        if (s_camera_buffer.canvas_buf[i] != NULL) {
            heap_caps_free(s_camera_buffer.canvas_buf[i]);
            s_camera_buffer.canvas_buf[i] = NULL;
        }
    }
    
    if (s_camera_buffer.jpg_buf != NULL) {
        heap_caps_free(s_camera_buffer.jpg_buf); 
        s_camera_buffer.jpg_buf = NULL;
    }
    ESP_LOGI(TAG, "Internal buffers cleaned up");
}

/* -------------------------------------------------------------------------- */
/* --- Unified Implementations (Frame Processing) --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Internal callback for processing a single dequeued camera frame.
 */
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, 
                                        uint32_t camera_buf_hes, uint32_t camera_buf_ves, 
                                        size_t camera_buf_len) {
    
    // 1. Camera Initialization Check
    if (!s_camera_state.flags.is_initialized) {
        s_camera_state.init_count++;
        if (s_camera_state.init_count >= CAMERA_INIT_FRAMES) {
            s_camera_state.flags.is_initialized = true;
            s_camera_state.init_count = 0;
            ESP_LOGI(TAG, "Camera initialized after %d frames", CAMERA_INIT_FRAMES);
        }
    }

    // 2. Scale and Rotate for Display (PPA operation)
    // Scale level 1 is used here, implying the output resolution matches the canvas size
    esp_err_t ret = app_image_process_video_frame(
        camera_buf, camera_buf_hes, camera_buf_ves,
        1, s_current_ppa_rotation,
        s_camera_buffer.canvas_buf[camera_buf_index], BSP_LCD_H_RES, BSP_LCD_V_RES,
        ALIGN_UP(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_data_cache_line_size)
    );
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to process frame (PPA): 0x%x", ret);
        return;
    }
    
    // 3. Resize and Grayscale for Optical Flow Input
    resize_frame_nearest(
        (const uint16_t*)s_camera_buffer.canvas_buf[camera_buf_index], 
        BSP_LCD_H_RES, BSP_LCD_V_RES,
        g_frame, 
        OPTFLOW_WIDTH, OPTFLOW_HEIGHT);

    // 4. Update Optical Flow/User Frame
    app_video_update_frame(g_frame, OPTFLOW_WIDTH, OPTFLOW_HEIGHT);
    
    // 5. Byte Swap for Display
    swap_rgb565_bytes(s_camera_buffer.canvas_buf[camera_buf_index], BSP_LCD_H_RES * BSP_LCD_V_RES);    
    
    // 6. Update Display
    update_display(s_camera_buffer.canvas_buf[camera_buf_index]);
}

/**
 * @brief Register the user callback for processed frame updates.
 */
static esp_err_t app_video_register_frame_update(frame_update_t frame_update) {
    if (frame_update == NULL) {
        ESP_LOGE(TAG, "Frame update callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_video_ctx.frame_update = frame_update;
    ESP_LOGI(TAG, "Frame update callback registered");
    return ESP_OK;
}

/**
 * @brief Execute the user callback.
 */
static void app_video_update_frame(uint8_t *frame, uint32_t width, uint32_t height) {
    s_video_ctx.frame_update(frame, width, height);
}

/**
 * @brief Register the internal callback for V4L2 frame processing.
 */
static esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb) {
    if (operation_cb == NULL) {
        ESP_LOGE(TAG, "Frame operation callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_video_ctx.user_frame_operation_cb = operation_cb;
    ESP_LOGI(TAG, "Frame operation callback registered");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* --- Public API Implementations --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Main initialization routine.
 */
esp_err_t app_video_stream_init(frame_update_t frame_update) {
    ESP_LOGI(TAG, "Starting video streaming initialization");

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
        goto cleanup_utils; 
    }

    // 4. Allocate all required internal buffers
    ret = initialize_buffers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buffers");
        goto cleanup_device;
    }

    // 5. Configure V4L2 buffers (MMAP mode: NULL for fb)
    ret = app_video_set_bufs(s_camera_buffer.video_cam_fd, EXAMPLE_CAM_BUF_NUM, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set video buffers: 0x%x", ret);
        goto cleanup_buffers;
    }

    // 6. Register frame callbacks
    ret = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame operation callback: 0x%x", ret);
        goto cleanup_buffers;
    }
    ret = app_video_register_frame_update(frame_update);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register frame update callback: 0x%x", ret);
        goto cleanup_buffers;
    }

    // 7. Start video streaming (VIDIOC_STREAMON)
    ret = video_stream_start(s_camera_buffer.video_cam_fd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream: 0x%x", ret);
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

/**
 * @brief Main deinitialization routine.
 */
esp_err_t app_video_stream_deinit(void) {
    ESP_LOGI(TAG, "Starting video stream deinitialization");
    
    // 1. Request task stop and wait for completion signal
    if (s_camera_buffer.video_cam_fd >= 0) {
        s_video_ctx.video_task_delete = true;
        // Wait for the external capture task (via app_video_stream_capture) to signal stop
        app_video_wait_video_stop(); 
    }
    
    // 2. Reset state and clean up buffers
    s_current_ppa_rotation = PPA_SRM_ROTATION_ANGLE_0;
    cleanup_buffers();
    
    // 3. Close video device (includes semaphore cleanup)
    if (s_camera_buffer.video_cam_fd >= 0) {
        app_video_close(s_camera_buffer.video_cam_fd);
        s_camera_buffer.video_cam_fd = -1;
    }
    
    // 4. Deinitialize video utilities
    app_video_utils_deinit();
    
    // 5. Reset camera state
    memset(&s_camera_state, 0, sizeof(s_camera_state));
    
    ESP_LOGI(TAG, "Video stream deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief External function to capture a frame (main loop body).
 */
void app_video_stream_capture(void) {
    int video_fd = s_camera_buffer.video_cam_fd;

    // 1. Receive frame
    if (video_receive_frame(video_fd) != ESP_OK) {
        return;
    }

    // 2. Process frame (calls camera_video_frame_operation)
    video_process_frame(video_fd);

    // 3. Return frame to driver
    if (video_free_frame(video_fd) != ESP_OK) {
        ESP_LOGI(TAG, "Video frame requeue failed");
    }

    // 4. Check for stream termination request
    if (s_video_ctx.video_task_delete) {
        ESP_LOGI(TAG, "Video stream task termination sequence initiated");
        s_video_ctx.video_task_delete = false;
        
        // Stop V4L2 stream and signal completion
        video_stream_stop(video_fd);
        if (s_video_ctx.video_stop_sem != NULL) {
            xSemaphoreGive(s_video_ctx.video_stop_sem);
        }
    }
}

// --- Getter/Setter Implementations ---

esp_err_t app_video_stream_set_flash_light(bool is_on) {
    s_camera_state.flags.is_flash_light_on = is_on;
    ESP_LOGI(TAG, "Flash light %s", is_on ? "ON" : "OFF");
    return ESP_OK;
}

bool app_video_stream_get_flash_light_state(void) {
    return s_camera_state.flags.is_flash_light_on;
}

int app_video_stream_get_video_fd(void) {
    return s_camera_buffer.video_cam_fd;
}

void app_video_stream_get_jpg_buf(uint8_t **buf, uint32_t *size) {
    *buf = s_camera_buffer.jpg_buf;
    *size = (uint32_t)s_camera_buffer.rx_buffer_size;
}

uint32_t app_video_get_buf_size(void) {
    // Assumes APP_VIDEO_FMT is RGB565 (2 bytes)
    uint32_t bytes_per_pixel = 2; 
    return s_video_ctx.camera_buf_hes * s_video_ctx.camera_buf_ves * bytes_per_pixel;
}