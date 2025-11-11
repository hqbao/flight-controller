#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <math.h> // Not strictly necessary since we only use positive floats for flooring (implicit cast to int)

// Helper function to safely get pixel value from source image with clamping
// This prevents reading out-of-bounds memory, especially important for x2/y2 coordinates.
static inline unsigned char get_src_pixel(
    const unsigned char *src_image, int w, int h, int x, int y) 
{
  // Clamp x coordinate to [0, w-1]
  if (x < 0) x = 0;
  if (x >= w) x = w - 1;

  // Clamp y coordinate to [0, h-1]
  if (y < 0) y = 0;
  if (y >= h) y = h - 1;
  
  return src_image[y * w + x];
}

/**
 * @brief Crops a region from the source image and resizes it to the destination size 
 * using Bilinear Interpolation.
 * * Assumes single-channel (grayscale) 8-bit unsigned char images.
 * * @param src_image Pointer to the source image data.
 * @param src_w Width of the source image.
 * @param src_h Height of the source image.
 * @param dst_image Pointer to the destination image buffer (must be pre-allocated to dst_w * dst_h).
 * @param dst_w Width of the destination image.
 * @param dst_h Height of the destination image.
 * @param crop_x X coordinate of the top-left corner of the crop box in the source image.
 * @param crop_y Y coordinate of the top-left corner of the crop box in the source image.
 * @param crop_w Width of the crop box.
 * @param crop_h Height of the crop box.
 * @return int 0 on success, -1 on invalid input, -2 if the clamped crop region is empty.
 */
int fast_crop_and_resize_bilinear(
    const unsigned char *src_image, int src_w, int src_h,
    unsigned char *dst_image, int dst_w, int dst_h,
    int crop_x, int crop_y, int crop_w, int crop_h) 
{
  // 1. Input Validation and Sanity Check
  if (!src_image || !dst_image || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0 || crop_w <= 0 || crop_h <= 0) {
    return -1;
  }

  // 2. Validate and Clamp crop region boundaries
  // We adjust the crop dimensions if the requested box goes outside the source image bounds.
  
  int new_crop_x = crop_x;
  int new_crop_y = crop_y;
  int new_crop_w = crop_w;
  int new_crop_h = crop_h;

  // Clamp x start and width
  if (new_crop_x < 0) {
    new_crop_w += new_crop_x;
    new_crop_x = 0;
  }
  if (new_crop_x + new_crop_w > src_w) {
    new_crop_w = src_w - new_crop_x;
  }
  
  // Clamp y start and height
  if (new_crop_y < 0) {
    new_crop_h += new_crop_y;
    new_crop_y = 0;
  }
  if (new_crop_y + new_crop_h > src_h) {
    new_crop_h = src_h - new_crop_y;
  }
  
  // Check if the resulting crop is valid
  if (new_crop_w <= 0 || new_crop_h <= 0) {
    // If the resulting crop is invalid, fill the destination with 0 (black) and return success
    memset(dst_image, 0, dst_w * dst_h);
    return -2;
  }

  // Use the clamped values for calculation
  crop_x = new_crop_x;
  crop_y = new_crop_y;
  crop_w = new_crop_w;
  crop_h = new_crop_h;

  // 3. Pre-calculate scaling factors
  // scale = (crop_size / dst_size) to map output coordinates [0, dst_size] to input [0, crop_size]
  const float scale_x = (float)crop_w / dst_w;
  const float scale_y = (float)crop_h / dst_h;

  // 4. Loop through destination image pixels
  for (int y_dst = 0; y_dst < dst_h; ++y_dst) {
    for (int x_dst = 0; x_dst < dst_w; ++x_dst) {
      
      // a. Map destination pixel (x_dst, y_dst) to floating-point source coordinates (x_src, y_src)
      // The coordinate is: (crop_offset) + (dst_coord * scale)
      const float x_src_float = (float)crop_x + (float)x_dst * scale_x;
      const float y_src_float = (float)crop_y + (float)y_dst * scale_y;
      
      // b. Find the four surrounding integer coordinates (x1, y1) is the top-left neighbor
      const int x1 = (int)x_src_float;
      const int y1 = (int)y_src_float;
      const int x2 = x1 + 1;
      const int y2 = y1 + 1;

      // c. Calculate fractional weights (a and b) for interpolation
      const float a = x_src_float - (float)x1; // x-fractional part
      const float b = y_src_float - (float)y1; // y-fractional part
      
      // d. Get the four source pixel values (Q11, Q21, Q12, Q22)
      // get_src_pixel handles clamping if x2/y2 go out of bounds (which can happen at the right/bottom edge)
      const float Q11 = (float)get_src_pixel(src_image, src_w, src_h, x1, y1); // (x1, y1)
      const float Q21 = (float)get_src_pixel(src_image, src_w, src_h, x2, y1); // (x2, y1)
      const float Q12 = (float)get_src_pixel(src_image, src_w, src_h, x1, y2); // (x1, y2)
      const float Q22 = (float)get_src_pixel(src_image, src_w, src_h, x2, y2); // (x2, y2)
      
      // e. Perform Bilinear Interpolation: 
      // P = (Q11*(1-a) + Q21*a)*(1-b) + (Q12*(1-a) + Q22*a)*b
      
      // Linear interpolation along X (top row, P1)
      const float P1 = Q11 * (1.0f - a) + Q21 * a;
      
      // Linear interpolation along X (bottom row, P2)
      const float P2 = Q12 * (1.0f - a) + Q22 * a;
      
      // Linear interpolation along Y (final pixel value)
      float P = P1 * (1.0f - b) + P2 * b;
      
      // f. Round to nearest integer and clamp to 8-bit range [0, 255]
      int final_pixel_value = (int)(P + 0.5f);
      
      // Final clamp (shouldn't be strictly necessary if source is [0, 255] but good practice)
      if (final_pixel_value < 0) final_pixel_value = 0;
      if (final_pixel_value > 255) final_pixel_value = 255;
      
      dst_image[y_dst * dst_w + x_dst] = (unsigned char)final_pixel_value;
    }
  }

  return 0; // Success
}
