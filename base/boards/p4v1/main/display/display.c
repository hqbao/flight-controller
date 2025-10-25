#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_check.h>
#include "display.h"
#include "overlay_manager.h"

static const char *TAG = "display";

#define LCD_CMD_BITS         (8)
#define LCD_PARAM_BITS       (8)
#define LCD_LEDC_CH          (1)
#define LVGL_TICK_PERIOD_MS  (5)
#define LVGL_MAX_SLEEP_MS    (1)

static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static lv_disp_t *disp = NULL;

// LCD panel initialization commands
typedef struct {
    int cmd;
    const void *data;
    size_t data_bytes;
    unsigned int delay_ms;
} lcd_init_cmd_t;

static const lcd_init_cmd_t vendor_specific_init[] = {
    {0x11, (uint8_t []){0x00}, 1, 120},
    {0xB2, (uint8_t []){0x0C, 0x0C, 0x00, 0x33, 0x33}, 5, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x36, (uint8_t []){0x00}, 1, 0},
    {0x3A, (uint8_t []){0x05}, 1, 0},
    {0xB7, (uint8_t []){0x35}, 1, 0},
    {0xBB, (uint8_t []){0x2D}, 1, 0},
    {0xC0, (uint8_t []){0x2C}, 1, 0},
    {0xC2, (uint8_t []){0x01}, 1, 0},
    {0xC3, (uint8_t []){0x15}, 1, 0},
    {0xC4, (uint8_t []){0x20}, 1, 0},
    {0xC6, (uint8_t []){0x0F}, 1, 0},
    {0xD0, (uint8_t []){0xA4, 0xA1}, 2, 0},
    {0xD6, (uint8_t []){0xA1}, 1, 0},
    {0xE0, (uint8_t []){0x70, 0x05, 0x0A, 0x0B, 0x0A, 0x27, 0x2F, 0x44, 0x47, 0x37, 0x14, 0x14, 0x29, 0x2F}, 14, 0},
    {0xE1, (uint8_t []){0x70, 0x07, 0x0C, 0x08, 0x08, 0x04, 0x2F, 0x33, 0x46, 0x18, 0x15, 0x15, 0x2B, 0x2D}, 14, 0},
    {0x21, (uint8_t []){0x00}, 1, 0},
    {0x29, (uint8_t []){0x00}, 1, 0},
    {0x2C, (uint8_t []){0x00}, 1, 0},
};

esp_err_t bsp_display_brightness_init(void) {
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = true
    };
    
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent) {
    if (brightness_percent > 100) brightness_percent = 100;
    if (brightness_percent < 0) brightness_percent = 0;

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void) {
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void) {
    return bsp_display_brightness_set(100);
}

static esp_err_t bsp_display_new(esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io) {
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    // Initialize SPI bus
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BSP_LCD_H_RES * 10 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    // Install panel IO
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = 80 * 1000 * 1000,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 3,
        .trans_queue_depth = 2,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, ret_io), err, TAG, "New panel IO failed");

    // Install LCD driver
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    // Send initialization commands
    const lcd_init_cmd_t *cmd = vendor_specific_init;
    uint16_t cmd_size = sizeof(vendor_specific_init) / sizeof(lcd_init_cmd_t);
    for (uint16_t i = 0; i < cmd_size; i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(*ret_io, cmd[i].cmd, cmd[i].data, cmd[i].data_bytes), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(cmd[i].delay_ms));
    }

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_invert_color(*ret_panel, true);
    return ret;

err:
    if (*ret_panel) esp_lcd_panel_del(*ret_panel);
    if (*ret_io) esp_lcd_panel_io_del(*ret_io);
    spi_bus_free(SPI2_HOST);
    return ret;
}

static lv_disp_t *bsp_display_lcd_init(void) {
    bsp_display_new(&panel_handle, &io_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // Add LCD screen to LVGL
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = 0,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = true,
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_disp_t *bsp_display_start(void) {
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 2,
        .task_stack = 4096,
        .task_affinity = 0,
        .timer_period_ms = LVGL_TICK_PERIOD_MS,
        .task_max_sleep_ms = LVGL_MAX_SLEEP_MS,
    };
    
    lvgl_port_init(&lvgl_cfg);
    disp = bsp_display_lcd_init();
    return disp;
}

bool bsp_display_lock(uint32_t timeout_ms) {
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void) {
    lvgl_port_unlock();
}

// UI elements
static lv_obj_t *ui_ScreenCamera;
static lv_obj_t *ui_PanelCanvas;

void init_display(void) {
    bsp_display_start();
    bsp_display_lock(0);

    // Initialize theme
    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, 
        lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);

    // Create UI
    ui_ScreenCamera = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenCamera, LV_OBJ_FLAG_SCROLLABLE);

    ui_PanelCanvas = lv_canvas_create(ui_ScreenCamera);
    lv_obj_set_width(ui_PanelCanvas, BSP_LCD_V_RES);
    lv_obj_set_height(ui_PanelCanvas, BSP_LCD_H_RES);
    lv_obj_set_align(ui_PanelCanvas, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_PanelCanvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_color(ui_PanelCanvas, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_PanelCanvas, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_disp_load_scr(ui_ScreenCamera);
    bsp_display_unlock();

    // Turn on backlight
    bsp_display_backlight_on();
}

void update_display(void *buffer) {
    bsp_display_lock(0);
    lv_canvas_set_buffer(ui_PanelCanvas, buffer, BSP_LCD_H_RES, BSP_LCD_V_RES, LV_IMG_CF_TRUE_COLOR);
    // lv_refr_now(NULL);
    bsp_display_unlock();
}
