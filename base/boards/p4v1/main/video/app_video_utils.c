#include <stdio.h>
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "driver/ppa.h"
#include "driver/jpeg_encode.h"
#include "app_video_utils.h"

static const char *TAG = "app_video_utils";

/* Static variables */
static ppa_client_handle_t s_ppa_handle = NULL;
static jpeg_encoder_handle_t s_jpeg_handle = NULL;

/* Scale configuration */
static const int s_scale_level_res[SCALE_LEVELS] = {960, 480, 240, 120};
static const uint32_t s_adj_resolution_width[SCALE_LEVELS] = {1920, 960, 480, 240};
static const uint32_t s_adj_resolution_height[SCALE_LEVELS] = {1080, 540, 270, 135};

/* Private function declarations */
static bool validate_scale_level(int scale_level);
static bool validate_magnification_factor(uint16_t factor);

/* Public function implementations */

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

esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, 
    size_t out_buf_size, ppa_srm_rotation_angle_t rotation_angle) {
    // Validate input parameters
    if (in_buf == NULL || out_buf == NULL) {
        ESP_LOGE(TAG, "Invalid buffer pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (in_width == 0 || in_height == 0 || out_width == 0 || out_height == 0) {
        ESP_LOGE(TAG, "Invalid dimensions");
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
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(s_ppa_handle, &srm_config);
}

esp_err_t app_image_process_magnify(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint16_t magnification_factor, uint8_t *out_buf, size_t out_buf_size) {
    // Validate parameters
    if (!validate_magnification_factor(magnification_factor)) {
        ESP_LOGE(TAG, "Invalid magnification factor: %d", magnification_factor);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (in_buf == NULL || out_buf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t crop_width = s_adj_resolution_width[magnification_factor - 1];
    uint32_t crop_height = s_adj_resolution_height[magnification_factor - 1];

    return app_image_process_scale_crop(
        in_buf, in_width, in_height,
        crop_width, crop_height,
        out_buf, in_width, in_height, out_buf_size,
        PPA_SRM_ROTATION_ANGLE_0
    );
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
    if (src_buf == NULL || out_buf == NULL || out_size == NULL) {
        ESP_LOGE(TAG, "Invalid buffer pointers");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (width == 0 || height == 0) {
        ESP_LOGE(TAG, "Invalid image dimensions: %" PRIu32 "x%" PRIu32, width, height);
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

/* Private function implementations */

static bool validate_scale_level(int scale_level) {
    return (scale_level >= 1 && scale_level <= SCALE_LEVELS);
}

static bool validate_magnification_factor(uint16_t factor) {
    return (factor >= 1 && factor <= SCALE_LEVELS);
}
