#ifndef MATRIX_H
#define MATRIX_H

#include "vector3d.h"

/* Bumped 7→16 for fusion6 ESKF (16-element error state). Inflates fusion2_t but
 * stays well within STM32H7 SRAM. fusion6 is the only client requiring >7. */
#define MATRIX_MAX_SIZE 16

/** Fixed-size matrix (max 16x16) with double precision */
typedef struct {
    double data[MATRIX_MAX_SIZE][MATRIX_MAX_SIZE];
    int rows;
    int cols;
} matrix_t;

/** Initialize matrix with given dimensions (zeroed) */
void matrix_init(matrix_t* m, int rows, int cols);

/** Set identity matrix of given size */
void matrix_eye(matrix_t* m, int size);

/** Matrix multiplication (result = a * b) */
void matrix_mult(matrix_t* result, const matrix_t* a, const matrix_t* b);

/** Optimized 3D vector multiply: result = m[0:3,0:3] * v.
 *  Avoids zeroing/looping over full MATRIX_MAX_SIZE storage in realtime paths. */
void matrix_mult_vec3(vector3d_t* result, const matrix_t* m, const vector3d_t* v);

/** Optimized 3D vector multiply: result = m[0:3,0:3]^T * v. */
void matrix_mult_transpose_vec3(vector3d_t* result, const matrix_t* m, const vector3d_t* v);

/** Matrix addition (result = a + b) */
void matrix_add(matrix_t* result, const matrix_t* a, const matrix_t* b);

/** Matrix subtraction (result = a - b) */
void matrix_sub(matrix_t* result, const matrix_t* a, const matrix_t* b);

/** Matrix transpose (result = a') */
void matrix_transpose(matrix_t* result, const matrix_t* a);

/** Matrix scalar multiplication (result = a * s) */
void matrix_scale(matrix_t* result, const matrix_t* a, double s);

/** Copy matrix (result = a) */
void matrix_copy(matrix_t* result, const matrix_t* a);

/** Invert a 3x3 matrix using cofactor expansion. Returns 1 on success, 0 if singular
 *  (|det| < 1e-12). Result and input may not alias. */
int matrix_invert_3x3(matrix_t* result, const matrix_t* a);

/** Invert a symmetric positive-definite matrix in-place using Cholesky decomposition
 *  (A = L L^T, then A^{-1} = L^{-T} L^{-1}). Returns 1 on success, 0 on failure
 *  (non-positive pivot encountered). Works for any N up to MATRIX_MAX_SIZE. */
int matrix_invert_spd(matrix_t* a);

/** Joseph form covariance update (numerically stable, preserves symmetry & PSD):
 *  P_new = (I - K*H) * P * (I - K*H)^T + K * R * K^T
 *  Used for ESKF/EKF measurement update. P must be N×N, K is N×M, H is M×N, R is M×M. */
void matrix_joseph_update(matrix_t* P_new, const matrix_t* P, const matrix_t* K,
                          const matrix_t* H, const matrix_t* R);

/** Force symmetry in-place: m = (m + m^T) / 2. m must be square. Used after every
 *  covariance update to suppress floating-point drift away from symmetry. */
void matrix_symmetrize(matrix_t* m);

/** Clamp diagonal entries from below: m[i][i] = max(m[i][i], floor). m must be square.
 *  Used to prevent covariance collapse to zero in well-observed states. */
void matrix_diag_floor(matrix_t* m, double floor);

#endif