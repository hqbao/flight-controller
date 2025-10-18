#ifndef VECTOR3D_H
#define VECTOR3D_H

typedef struct {
    double x, y, z;
} vector3d_t;

// Initialize a vector
void vector3d_init(vector3d_t* v, double x, double y, double z);

// Assign a vector
void vector3d_set(vector3d_t* result, const vector3d_t* v);

// Add two vectors (a + b)
void vector3d_add(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

// Subtract two vectors (a - b)
void vector3d_sub(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

// Multiply vector by scalar
void vector3d_scale(vector3d_t* result, const vector3d_t* v, double s);

// Dot product of two vectors
double vector3d_dot(const vector3d_t* a, const vector3d_t* b);

// Cross product of two vectors (a × b)
void vector3d_cross(vector3d_t* result, const vector3d_t* a, const vector3d_t* b);

// Norm (magnitude) of vector
double vector3d_norm(const vector3d_t* v);

// Normalize vector
void vector3d_normalize(vector3d_t* result, const vector3d_t* v);

// Follow a vector
void vector3d_follow(vector3d_t *v1, vector3d_t *v2, double speed);

#endif
