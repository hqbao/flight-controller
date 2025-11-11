#ifndef IMAGE_UTIL_H
#define IMAGE_UTIL_H

int fast_crop_and_resize_bilinear(
    const unsigned char *src_image, int src_w, int src_h,
    unsigned char *dst_image, int dst_w, int dst_h,
    int crop_x, int crop_y, int crop_w, int crop_h);

#endif