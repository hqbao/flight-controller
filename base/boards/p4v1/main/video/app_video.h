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

/* --- Constants and Limits --- */

// Device and Buffer Configuration
#define EXAMPLE_CAM_DEV_PATH            (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define EXAMPLE_CAM_BUF_NUM             (2)
#define APP_VIDEO_FMT                   (APP_VIDEO_FMT_RGB565)
#define MAX_BUFFER_COUNT                (6)
#define MIN_BUFFER_COUNT                (2)

// Task Configuration
#define VIDEO_TASK_STACK_SIZE           (4 * 1024)
#define VIDEO_TASK_PRIORITY             (6)

// Image Processing and Streaming Defaults
#define SCALE_LEVELS                    4
#define JPEG_COMPRESSION_RATIO          5
#define CAMERA_INIT_FRAMES              50
#define SHARED_PHOTO_BUF_WIDTH          1280
#define SHARED_PHOTO_BUF_HEIGHT         720
#define PHOTO_WIDTH_1080P               1920
#define PHOTO_HEIGHT_1080P              1080
#define PHOTO_HEIGHT_1088P              1088 // JPEG encoder aligned height

// Helper macro (from app_video_stream.h)
#define ALIGN_UP(num, align)            (((num) + ((align) - 1)) & ~((align) - 1))

/* --- Type Definitions --- */

/**
 * @brief Video pixel formats (V4L2_PIX_FMT_*)
 */
typedef enum {
    APP_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} video_fmt_t;

typedef void (*frame_update_t)(uint8_t *buffer, uint32_t width, uint32_t height);

/**
 * @brief Initialize video streaming application (combines utils, main, open, set_bufs, start_task)
 */
esp_err_t app_video_stream_init(frame_update_t frame_update);

void app_video_stream_capture(void);

/**
 * @brief Deinitialize video streaming application (cleans up everything)
 */
esp_err_t app_video_stream_deinit(void);

#ifdef __cplusplus
}
#endif
#endif /* APP_VIDEO_UNIFIED_H */
