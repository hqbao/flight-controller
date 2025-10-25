#include <math.h>
#include "overlay_manager.h"

// Set pixel with bounds checking
void set_pixel(uint16_t *buffer, uint32_t width, uint32_t height,
               int x, int y, uint16_t color) {
    if (x >= 0 && x < (int)width && y >= 0 && y < (int)height) {
        buffer[y * width + x] = color;
    }
}

// Bresenham's line algorithm
void draw_line(uint16_t *buffer, uint32_t width, uint32_t height,
               int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (1) {
        set_pixel(buffer, width, height, x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void draw_polyline(uint16_t *buffer, uint32_t width, uint32_t height,
                   const int *points, int point_count, uint16_t color) {
    for (int i = 0; i < point_count - 1; i++) {
        int x0 = points[i * 2];
        int y0 = points[i * 2 + 1];
        int x1 = points[(i + 1) * 2];
        int y1 = points[(i + 1) * 2 + 1];
        
        draw_line(buffer, width, height, x0, y0, x1, y1, color);
    }
}

void draw_rectangle(uint16_t *buffer, uint32_t width, uint32_t height,
                    int x0, int y0, int x1, int y1, uint16_t color, bool filled) {
    if (filled) {
        // Draw filled rectangle
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                set_pixel(buffer, width, height, x, y, color);
            }
        }
    } else {
        // Draw rectangle outline
        draw_line(buffer, width, height, x0, y0, x1, y0, color); // Top
        draw_line(buffer, width, height, x1, y0, x1, y1, color); // Right
        draw_line(buffer, width, height, x1, y1, x0, y1, color); // Bottom
        draw_line(buffer, width, height, x0, y1, x0, y0, color); // Left
    }
}

// Midpoint circle algorithm
void draw_circle(uint16_t *buffer, uint32_t width, uint32_t height,
                 int center_x, int center_y, int radius, uint16_t color, bool filled) {
    if (filled) {
        // Simple filled circle (not perfect but fast)
        for (int y = -radius; y <= radius; y++) {
            for (int x = -radius; x <= radius; x++) {
                if (x*x + y*y <= radius*radius) {
                    set_pixel(buffer, width, height, center_x + x, center_y + y, color);
                }
            }
        }
    } else {
        // Circle outline using midpoint algorithm
        int x = radius;
        int y = 0;
        int err = 0;
        
        while (x >= y) {
            set_pixel(buffer, width, height, center_x + x, center_y + y, color);
            set_pixel(buffer, width, height, center_x + y, center_y + x, color);
            set_pixel(buffer, width, height, center_x - y, center_y + x, color);
            set_pixel(buffer, width, height, center_x - x, center_y + y, color);
            set_pixel(buffer, width, height, center_x - x, center_y - y, color);
            set_pixel(buffer, width, height, center_x - y, center_y - x, color);
            set_pixel(buffer, width, height, center_x + y, center_y - x, color);
            set_pixel(buffer, width, height, center_x + x, center_y - y, color);
            
            y++;
            err += 1 + 2*y;
            if (2*(err - x) + 1 > 0) {
                x--;
                err += 1 - 2*x;
            }
        }
    }
}

void draw_arrow(uint16_t *buffer, uint32_t width, uint32_t height,
                int x0, int y0, int x1, int y1, uint16_t color, int head_size) {
    // Draw the main line
    draw_line(buffer, width, height, x0, y0, x1, y1, color);
    
    // Calculate arrowhead
    double angle = atan2(y1 - y0, x1 - x0);
    
    // Arrowhead lines
    int x2 = x1 - head_size * cos(angle - M_PI/6);
    int y2 = y1 - head_size * sin(angle - M_PI/6);
    int x3 = x1 - head_size * cos(angle + M_PI/6);
    int y3 = y1 - head_size * sin(angle + M_PI/6);
    
    draw_line(buffer, width, height, x1, y1, x2, y2, color);
    draw_line(buffer, width, height, x1, y1, x3, y3, color);
}