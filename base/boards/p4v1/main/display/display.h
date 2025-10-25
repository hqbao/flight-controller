#ifndef DISPLAY_H
#define DISPLAY_H

#include <esp_lcd_types.h>
#include <esp_lvgl_port.h>

// Display configuration
#define BSP_LCD_BITS_PER_PIXEL      (16)
#define BSP_LCD_H_RES               (240)
#define BSP_LCD_V_RES               (240)
#define BSP_LCD_DRAW_BUFF_SIZE      (BSP_LCD_H_RES * BSP_LCD_V_RES)

// GPIO definitions
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_16)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_17)
#define BSP_LCD_SPI_CS        (GPIO_NUM_18)
#define BSP_LCD_DC            (GPIO_NUM_19)
#define BSP_LCD_RST           (GPIO_NUM_15)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_20)

// Function declarations
void init_display(void);
void update_display(void *buffer);
esp_err_t bsp_display_brightness_set(int brightness_percent);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);

#endif