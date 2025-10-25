#ifndef APP_VIDEO_UTILS_H
#define APP_VIDEO_UTILS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/ppa.h"
#include "display.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Constants */
#define SCALE_LEVELS            4

/**
 * @brief Initialize video utilities
 * 
 * Initializes PPA (Pixel Processing Accelerator) and JPEG encoder
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_video_utils_init(void);

/**
 * @brief Deinitialize video utilities
 * 
 * Cleans up PPA and JPEG encoder resources
 * 
 * @return ESP_OK on success
 */
esp_err_t app_video_utils_deinit(void);

/**
 * @brief Perform image scaling, cropping, rotation and mirroring
 * 
 * @param in_buf Input image buffer
 * @param in_width Input image width
 * @param in_height Input image height
 * @param crop_width Crop region width
 * @param crop_height Crop region height
 * @param out_buf Output image buffer
 * @param out_width Output image width
 * @param out_height Output image height
 * @param out_buf_size Output buffer size
 * @param rotation_angle Rotation angle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, 
    size_t out_buf_size, ppa_srm_rotation_angle_t rotation_angle);

/**
 * @brief Perform image magnification processing
 * 
 * @param in_buf Input image buffer
 * @param in_width Input image width
 * @param in_height Input image height
 * @param magnification_factor Magnification factor (1-4)
 * @param out_buf Output image buffer
 * @param out_buf_size Output buffer size
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_image_process_magnify(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint16_t magnification_factor, uint8_t *out_buf, size_t out_buf_size);

/**
 * @brief Process video frame for display
 * 
 * @param in_buf Input image buffer
 * @param in_width Input image width
 * @param in_height Input image height
 * @param scale_level Scale level (1-4)
 * @param rotation_angle Rotation angle for compensation
 * @param out_buf Output image buffer
 * @param out_buf_size Output buffer size
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_image_process_video_frame(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    int scale_level, ppa_srm_rotation_angle_t rotation_angle,
    uint8_t *out_buf, uint32_t out_height, uint32_t out_width, size_t out_buf_size);

/**
 * @brief Encode RGB565 image to JPEG format
 * 
 * @param src_buf Source image buffer in RGB565 format
 * @param width Image width
 * @param height Image height
 * @param quality JPEG quality (0-100)
 * @param out_buf Output JPEG buffer
 * @param out_buf_size Size of output buffer
 * @param out_size Pointer to store the actual JPEG size
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_image_encode_jpeg(
    uint8_t *src_buf, uint32_t width, uint32_t height, uint8_t quality,
    uint8_t *out_buf, size_t out_buf_size, uint32_t *out_size);

/**
 * @brief Swap RGB565 bytes for correct display format
 * 
 * @param buffer RGB565 buffer to process
 * @param pixel_count Number of pixels in the buffer
 */
void swap_rgb565_bytes(uint16_t *buffer, int pixel_count);

#ifdef __cplusplus
}
#endif

#endif /* APP_VIDEO_UTILS_H */