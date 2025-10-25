#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "app_video.h"

static const char *TAG = "app_video";

/**
 * @brief Video application context structure
 */
typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    struct v4l2_buffer v4l2_buf;
    uint8_t camera_mem_mode;
    app_video_frame_operation_cb_t user_frame_operation_cb;
    TaskHandle_t video_stream_task_handle;
    bool video_task_delete;
    SemaphoreHandle_t video_stop_sem;
} app_video_context_t;

static app_video_context_t s_video_ctx;

/* Private function declarations */
static esp_err_t video_stream_start(int video_fd);
static esp_err_t video_stream_stop(int video_fd);
static esp_err_t video_receive_frame(int video_fd);
static esp_err_t video_free_frame(int video_fd);
static void video_process_frame(int video_fd);

/* Public function implementations */

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
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %" PRIu32 " exceeds maximum %d", fb_num, MAX_BUFFER_COUNT);
        return ESP_ERR_INVALID_SIZE;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %" PRIu32 " below minimum %d", fb_num, MIN_BUFFER_COUNT);
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
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %d exceeds maximum %d", fb_num, MAX_BUFFER_COUNT);
        return ESP_ERR_INVALID_SIZE;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "Buffer count %d below minimum %d", fb_num, MIN_BUFFER_COUNT);
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

esp_err_t app_video_stream_task_start(int video_fd) {
    // Start video stream
    esp_err_t ret = video_stream_start(video_fd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start video stream");
        return ret;
    }

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

esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb) {
    if (operation_cb == NULL) {
        ESP_LOGE(TAG, "Frame operation callback is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_video_ctx.user_frame_operation_cb = operation_cb;
    ESP_LOGI(TAG, "Frame operation callback registered");
    return ESP_OK;
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

/* Private function implementations */

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
        ESP_LOGE(TAG, "Failed to dequeue video frame");
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
