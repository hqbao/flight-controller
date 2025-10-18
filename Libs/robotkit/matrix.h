#ifndef MATRIX_H
#define MATRIX_H

#define MATRIX_MAX_SIZE 4

typedef struct {
    double data[MATRIX_MAX_SIZE][MATRIX_MAX_SIZE];
    int rows;
    int cols;
} matrix_t;

// Initialize matrix with given dimensions
void matrix_init(matrix_t* m, int rows, int cols);

// Set identity matrix
void matrix_eye(matrix_t* m, int size);

// Matrix multiplication (result = a * b)
void matrix_mult(matrix_t* result, const matrix_t* a, const matrix_t* b);

// Matrix addition (result = a + b)
void matrix_add(matrix_t* result, const matrix_t* a, const matrix_t* b);

// Matrix subtraction (result = a - b)
void matrix_sub(matrix_t* result, const matrix_t* a, const matrix_t* b);

// Matrix transpose (result = a')
void matrix_transpose(matrix_t* result, const matrix_t* a);

// Matrix scalar multiplication
void matrix_scale(matrix_t* result, const matrix_t* a, double s);

// Copy matrix (result = a)
void matrix_copy(matrix_t* result, const matrix_t* a);

#endif