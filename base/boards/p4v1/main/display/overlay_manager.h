#ifndef VECTOR_DRAW_H
#define VECTOR_DRAW_H

#include <stdint.h>
#include <stdbool.h>

// Colors (RGB565)
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_BLACK     0x0000
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF

/**
 * @brief Draw a line using Bresenham's algorithm
 * 
 * @param buffer Frame buffer (RGB565)
 * @param width Buffer width
 * @param height Buffer height
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color Line color (RGB565)
 */
void draw_line(uint16_t *buffer, uint32_t width, uint32_t height,
               int x0, int y0, int x1, int y1, uint16_t color);

/**
 * @brief Draw multiple connected lines (polyline)
 * 
 * @param buffer Frame buffer
 * @param width Buffer width
 * @param height Buffer height
 * @param points Array of points {x0,y0,x1,y1,...,xn,yn}
 * @param point_count Number of points in the array
 * @param color Line color
 */
void draw_polyline(uint16_t *buffer, uint32_t width, uint32_t height,
                   const int *points, int point_count, uint16_t color);

/**
 * @brief Draw a rectangle
 * 
 * @param buffer Frame buffer
 * @param width Buffer width
 * @param height Buffer height
 * @param x0 Top-left X
 * @param y0 Top-left Y
 * @param x1 Bottom-right X
 * @param y1 Bottom-right Y
 * @param color Rectangle color
 * @param filled True for filled rectangle
 */
void draw_rectangle(uint16_t *buffer, uint32_t width, uint32_t height,
                    int x0, int y0, int x1, int y1, uint16_t color, bool filled);

/**
 * @brief Draw a circle using midpoint algorithm
 * 
 * @param buffer Frame buffer
 * @param width Buffer width
 * @param height Buffer height
 * @param center_x Circle center X
 * @param center_y Circle center Y
 * @param radius Circle radius
 * @param color Circle color
 * @param filled True for filled circle
 */
void draw_circle(uint16_t *buffer, uint32_t width, uint32_t height,
                 int center_x, int center_y, int radius, uint16_t color, bool filled);

/**
 * @brief Draw an arrow (line with arrowhead)
 * 
 * @param buffer Frame buffer
 * @param width Buffer width
 * @param height Buffer height
 * @param x0 Start X
 * @param y0 Start Y
 * @param x1 End X
 * @param y1 End Y
 * @param color Arrow color
 * @param head_size Arrowhead size
 */
void draw_arrow(uint16_t *buffer, uint32_t width, uint32_t height,
                int x0, int y0, int x1, int y1, uint16_t color, int head_size);

/**
 * @brief Set a single pixel (with bounds checking)
 * 
 * @param buffer Frame buffer
 * @param width Buffer width
 * @param height Buffer height
 * @param x X coordinate
 * @param y Y coordinate
 * @param color Pixel color
 */
void set_pixel(uint16_t *buffer, uint32_t width, uint32_t height,
               int x, int y, uint16_t color);

#endif /* VECTOR_DRAW_H */