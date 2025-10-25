#ifndef APP_VIDEO_H
#define APP_VIDEO_H

#include <esp_err.h>
#include <driver/i2c_master.h>
#include <linux/videodev2.h>
#include <esp_video_device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants */
#define EXAMPLE_CAM_DEV_PATH            (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define EXAMPLE_CAM_BUF_NUM             (2)
#define APP_VIDEO_FMT                   (APP_VIDEO_FMT_RGB565)

/* Video buffer limits */
#define MAX_BUFFER_COUNT                (6)
#define MIN_BUFFER_COUNT                (2)

/* Task configuration */
#define VIDEO_TASK_STACK_SIZE           (4 * 1024)
#define VIDEO_TASK_PRIORITY             (6)

/**
 * @brief Video pixel formats
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

/**
 * @brief Frame operation callback function type
 */
typedef void (*app_video_frame_operation_cb_t)(uint8_t *camera_buf, 
                                              uint8_t camera_buf_index, 
                                              uint32_t camera_buf_hes, 
                                              uint32_t camera_buf_ves, 
                                              size_t camera_buf_len);

/**
 * @brief Initialize the video camera system
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_video_main(void);

/**
 * @brief Open and configure video capture device
 *
 * @param dev Device path (e.g., "/dev/video0")
 * @param init_fmt Desired pixel format
 * @return File descriptor on success, -1 on failure
 */
int app_video_open(char *dev, video_fmt_t init_fmt);

/**
 * @brief Set up video capture buffers
 *
 * @param video_fd Video device file descriptor
 * @param fb_num Number of frame buffers
 * @param fb Array of user-provided frame buffers (optional)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb);

/**
 * @brief Retrieve allocated video capture buffers
 *
 * @param fb_num Number of frame buffers to retrieve
 * @param fb Array to receive frame buffer pointers
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_video_get_bufs(int fb_num, void **fb);

/**
 * @brief Get video buffer size based on format and resolution
 *
 * @return Buffer size in bytes
 */
uint32_t app_video_get_buf_size(void);

/**
 * @brief Start video streaming task
 *
 * @param video_fd Video device file descriptor
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_video_stream_task_start(int video_fd);
void app_video_capture(int video_fd);

/**
 * @brief Stop video streaming task
 *
 * @param video_fd Video device file descriptor
 * @return ESP_OK on success
 */
esp_err_t app_video_stream_task_stop(int video_fd);

/**
 * @brief Wait for video stream task to complete
 *
 * @return ESP_OK on success
 */
esp_err_t app_video_wait_video_stop(void);

/**
 * @brief Register frame processing callback
 *
 * @param operation_cb Callback function for frame processing
 * @return ESP_OK on success
 */
esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb);

/**
 * @brief Close video device
 *
 * @param video_fd Video device file descriptor
 * @return ESP_OK on success
 */
esp_err_t app_video_close(int video_fd);

#ifdef __cplusplus
}
#endif
#endif /* APP_VIDEO_H */