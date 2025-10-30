//
//  mat.h
//  opticalflow
//
//  Created by Bao on 31/8/23.
//

#ifndef mat_h
#define mat_h

#include <stdio.h>
#include <stdint.h>

typedef struct {
    uint16_t w;
    uint16_t h;
    float *buf;
} matrix_t;

matrix_t* filter_hori(void);
matrix_t* filter_vert(void);
matrix_t* filter_prev(void);
matrix_t* filter_curr(void);

matrix_t* mat_alloc(void);

/**
 * Create new matrix from a buffer. This method will copy the input
 * buffer to the new buffer created. Later all, this created matrix
 * need to be released
 * If the input frame is NULL, a zeros matrix will be created
 */
matrix_t* mat_create(const uint8_t *frame, uint16_t width, uint16_t height);

void mat_print(const matrix_t* mat);

void mat_set(matrix_t* mat, const uint8_t *frame);
void mat_inv2x2(matrix_t *rmat);
void mat_trans(const matrix_t *mat, matrix_t *rmat);
void mat_flatten(matrix_t *rmat);
void mat_stack(const matrix_t *mat1, const matrix_t *mat2, matrix_t *rmat);
void mat_add(const matrix_t *mat1, const matrix_t *mat2, matrix_t *rmat);
void mat_negative(const matrix_t *mat1, matrix_t *rmat);
void mat_dot(const matrix_t *mat1, const matrix_t *mat2, matrix_t *rmat);
void mat_conv(const matrix_t *mat, const matrix_t *filter, matrix_t *rmat);
void mat_reset_all(void);

#endif /* mat_h */
