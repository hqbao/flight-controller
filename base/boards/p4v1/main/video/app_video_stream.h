#ifndef APP_VIDEO_STREAM_H
#define APP_VIDEO_STREAM_H

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/ppa.h>

/* Resolution constants */
#define PHOTO_WIDTH_480P    640
#define PHOTO_HEIGHT_480P   480
#define PHOTO_WIDTH_720P    1280
#define PHOTO_HEIGHT_720P   720
#define PHOTO_WIDTH_1080P   1920
#define PHOTO_HEIGHT_1080P  1080

/* JPEG encoder aligned height (16-byte aligned) */
#define PHOTO_HEIGHT_1088P  1088

/* Constants */
#define JPEG_COMPRESSION_RATIO          5
#define CAMERA_INIT_FRAMES              50
#define SHARED_PHOTO_BUF_WIDTH          1280
#define SHARED_PHOTO_BUF_HEIGHT         720

// Image processing defaults
#define DEFAULT_CONTRAST_PERCENT        53
#define DEFAULT_SATURATION_PERCENT      63
#define DEFAULT_BRIGHTNESS_PERCENT      54
#define DEFAULT_HUE_PERCENT             2

// Helper macro
#define ALIGN_UP(num, align)            (((num) + ((align) - 1)) & ~((align) - 1))

/**
 * @brief Initialize video streaming application
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_video_stream_init(void);

/**
 * @brief Deinitialize video streaming application
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_video_stream_deinit(void);

/**
 * @brief Set flash light state
 * 
 * @param is_on Whether to turn flash on or off
 * @return ESP_OK on success
 */
esp_err_t app_video_stream_set_flash_light(bool is_on);

/**
 * @brief Get flash light state
 * 
 * @return true if flash light is on, false otherwise
 */
bool app_video_stream_get_flash_light_state(void);

/**
 * @brief Get interval photo state
 * 
 * @return true if interval photo is active, false otherwise
 */
bool app_video_stream_get_interval_photo_state(void);

/**
 * @brief Get current interval minutes
 * 
 * @return Current interval in minutes
 */
uint16_t app_video_stream_get_current_interval_minutes(void);

/**
 * @brief Get video file descriptor
 * 
 * @return Video camera file descriptor
 */
int app_video_stream_get_video_fd(void);

/**
 * @brief Get scaled camera buffer
 * 
 * @param buf Pointer to store buffer address
 * @param size Pointer to store buffer size
 */
void app_video_stream_get_scaled_camera_buf(uint8_t **buf, uint32_t *size);

/**
 * @brief Get JPEG buffer
 * 
 * @param buf Pointer to store buffer address
 * @param size Pointer to store buffer size
 */
void app_video_stream_get_jpg_buf(uint8_t **buf, uint32_t *size);

/**
 * @brief Get shared photo buffer for record and photo modules
 * 
 * @param buf Pointer to store buffer address
 * @param size Pointer to store buffer size
 */
void app_video_stream_get_shared_photo_buf(uint8_t **buf, uint32_t *size);

#endif /* APP_VIDEO_STREAM_H */