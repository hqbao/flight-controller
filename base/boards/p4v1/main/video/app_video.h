#ifndef APP_VIDEO_UNIFIED_H
#define APP_VIDEO_UNIFIED_H

#include <esp_err.h>
#include <driver/i2c_master.h>
#include <linux/videodev2.h>
#include <esp_video_device.h>
#include <driver/ppa.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* --- Configuration Constants and Limits --- */
/* -------------------------------------------------------------------------- */

// Device and Buffer Configuration
#define EXAMPLE_CAM_DEV_PATH            (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define EXAMPLE_CAM_BUF_NUM             (2)
#define APP_VIDEO_FMT                   (APP_VIDEO_FMT_RGB565)
#define MAX_BUFFER_COUNT                (6)
#define MIN_BUFFER_COUNT                (2)

// Task Configuration (Kept for external user reference)
#define VIDEO_TASK_STACK_SIZE           (4 * 1024)
#define VIDEO_TASK_PRIORITY             (6)

// Image Processing and Streaming Defaults
#define SCALE_LEVELS                    4
#define JPEG_COMPRESSION_RATIO          5
#define CAMERA_INIT_FRAMES              50
#define PHOTO_WIDTH_1080P               1920
#define PHOTO_HEIGHT_1080P              1080
#define PHOTO_HEIGHT_1088P              1088 // JPEG encoder 16-byte aligned height for 1080P

// Optical Flow Configuration
#define OPTFLOW_WIDTH                   70
#define OPTFLOW_HEIGHT                  70

// Helper macro for cache alignment
#define ALIGN_UP(num, align)            (((num) + ((align) - 1)) & ~((align) - 1))

/* -------------------------------------------------------------------------- */
/* --- Type Definitions --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Video pixel formats, mapped to V4L2_PIX_FMT_*.
 */
typedef enum {
    APP_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P, // Planar 4:2:2
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,  // Planar 4:2:0
} video_fmt_t;

/**
 * @brief Callback function for updating a processed frame (e.g., optical flow input).
 */
typedef void (*frame_update_t)(uint8_t *buffer, uint32_t width, uint32_t height);

/* -------------------------------------------------------------------------- */
/* --- Public API: Core Streaming Functions --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initializes the video streaming application.
 */
esp_err_t app_video_stream_init(frame_update_t frame_update);

/**
 * @brief Deinitializes the video streaming application.
 */
esp_err_t app_video_stream_deinit(void);

/**
 * @brief Captures and processes a single frame from the video device.
 *
 * This function should be called repeatedly from an external video stream task.
 * It dequeues a buffer, processes it, and requeues it.
 */
void app_video_stream_capture(void);

/* -------------------------------------------------------------------------- */
/* --- Public API: Utility Functions (JPEG, Flash, Getters) --- */
/* -------------------------------------------------------------------------- */

/**
 * @brief Encodes a raw RGB565 buffer into a JPEG stream.
 */
esp_err_t app_image_encode_jpeg(
    uint8_t *src_buf, uint32_t width, uint32_t height, uint8_t quality,
    uint8_t *out_buf, size_t out_buf_size, uint32_t *out_size);

/**
 * @brief Sets the state of the flash light.
 */
esp_err_t app_video_stream_set_flash_light(bool is_on);

/**
 * @brief Gets the current flash light state.
 */
bool app_video_stream_get_flash_light_state(void);

/**
 * @brief Gets the file descriptor of the video device.
 */
int app_video_stream_get_video_fd(void);

/**
 * @brief Gets a pointer and size for the JPEG output buffer.
 */
void app_video_stream_get_jpg_buf(uint8_t **buf, uint32_t *size);

#ifdef __cplusplus
}
#endif
#endif /* APP_VIDEO_UNIFIED_H */