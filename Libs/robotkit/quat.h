#ifndef QUAT_H
#define QUAT_H

#include "vector3d.h"
#include "matrix.h"

typedef struct {
    float w, x, y, z;
} quaternion_t;

// Initialize quaternion
void quat_init(quaternion_t* q, float w, float x, float y, float z);

// Set quaternion
void quat_set(quaternion_t* q, const quaternion_t* q_new);

// Add quaternion
void quat_add(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

// Create identity quaternion
void quat_identity(quaternion_t* q);

// Normalize quaternion
void quat_normalize(quaternion_t* result, const quaternion_t* q);

// Quaternion multiplication (result = a * b)
void quat_mult(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

// Quaternion conjugate
void quat_conjugate(quaternion_t* result, const quaternion_t* q);

// Create quaternion from axis-angle
void quat_from_axis_angle(quaternion_t* q, const vector3d_t* axis, float angle);

// Quaternion from rotations
void quat_from_euler(quaternion_t *result, float roll, float pitch, float yaw);

// Convert quaternion to rotation matrix (3x3)
void quat_to_rot_matrix(matrix_t* rot, const quaternion_t* q);

// Rotate vector by quaternion (v' = q * v * q^-1)
void quat_rotate_vector(vector3d_t* result, const quaternion_t* q, const vector3d_t* v);

// Measurement function h(q): Predicts accelerometer reading from quaternion state
void quat_to_accel(vector3d_t *a, const quaternion_t *q, float g);

#endif
