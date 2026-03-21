#ifndef VECTOR3D_H
#define VECTOR3D_H

/** 3D vector with double precision components */
typedef struct {
    double x, y, z;
} vector3d_t;

/** Initialize a vector with x, y, z components */
void vector3d_init(vector3d_t* v, double x, double y, double z);

/** Copy vector (result = v) */
void vector3d_set(vector3d_t* result, const vector3d_t* v);

/** Add two vectors (result = a + b) */
void vector3d_add(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

/** Subtract two vectors (result = a - b) */
void vector3d_sub(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

/** Multiply vector by scalar (result = v * s) */
void vector3d_scale(vector3d_t* result, const vector3d_t* v, double s);

/** Dot product of two vectors */
double vector3d_dot(const vector3d_t* a, const vector3d_t* b);

/** Cross product (result = a × b) */
void vector3d_cross(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

/** Euclidean norm (magnitude) of vector */
double vector3d_norm(const vector3d_t* v);

/** Normalize vector to unit length */
void vector3d_normalize(vector3d_t* result, const vector3d_t* v);

/** Interpolate v1 toward v2 at given speed (low-pass filter) */
void vector3d_follow(vector3d_t *v1, const vector3d_t *v2, double speed);

#endif
