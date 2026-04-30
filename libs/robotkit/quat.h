#ifndef QUAT_H
#define QUAT_H

#include "vector3d.h"
#include "matrix.h"

/** Quaternion with float precision components (w, x, y, z) */
typedef struct {
    float w, x, y, z;
} quaternion_t;

/** Initialize quaternion with w, x, y, z components */
void quat_init(quaternion_t* q, float w, float x, float y, float z);

/** Copy quaternion (q = q_new) */
void quat_set(quaternion_t* q, const quaternion_t* q_new);

/** Add two quaternions (result = a + b) */
void quat_add(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

/** Set quaternion to identity (1, 0, 0, 0) */
void quat_identity(quaternion_t* q);

/** Normalize quaternion to unit length */
void quat_normalize(quaternion_t* result, const quaternion_t* q);

/** Quaternion multiplication (result = a * b, Hamilton product) */
void quat_mult(quaternion_t* result, const quaternion_t* a, const quaternion_t* b);

/** Quaternion conjugate (q* = w, -x, -y, -z) */
void quat_conjugate(quaternion_t* result, const quaternion_t* q);

/** Create quaternion from axis-angle representation */
void quat_from_axis_angle(quaternion_t* q, const vector3d_t* axis, float angle);

/** Create quaternion from Euler angles (roll, pitch, yaw in radians) */
void quat_from_euler(quaternion_t *result, float roll, float pitch, float yaw);

/** Convert quaternion to 3x3 rotation matrix */
void quat_to_rot_matrix(matrix_t* rot, const quaternion_t* q);

/** Rotate vector by quaternion (v' = q * v * q^-1) */
void quat_rotate_vector(vector3d_t* result, const quaternion_t* q, const vector3d_t* v);

/** Convert quaternion to Euler angles (roll, pitch, yaw in radians) */
void quat_to_euler(vector3d_t* euler, const quaternion_t* q);

/** Predict accelerometer reading from quaternion (NED: v_pred = -R(q)*[0,0,1]*g) */
void quat_to_accel(vector3d_t *a, const quaternion_t *q, float g);

/** Create quaternion from rotation vector φ (axis * angle, radians). */
void quat_from_rotation_vector(quaternion_t *q, double phi_x, double phi_y, double phi_z);

/** Compute rotation quaternion from vector u to vector v */
void quat_from_two_vectors(quaternion_t* q, const vector3d_t* u, const vector3d_t* v);

#endif
