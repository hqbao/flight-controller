#ifndef FILTER1_H
#define FILTER1_H

#include "vector3d.h"
#include "quat.h"

typedef struct {
    quaternion_t q;
    vector3d_t a;
    vector3d_t a_smooth;
    vector3d_t v_pred;
    vector3d_t v_true;
    vector3d_t v_linear_acc;
    vector3d_t v_linear_acc_smooth;
    double freq;
    double k0;
    double k1;
    double k2;
    double accel_scale;
    double accel_uncertainty;
    double min_linear_accel;
    double max_linear_accel;
    double linear_accel;
    char no_correction;
} filter1_t;

void filter1_init(filter1_t *f, double k0, double k1, double freq);
void filter1_remove_linear_accel(filter1_t *f, double k2, double min_linear_accel, double max_linear_accel, double accel_scale);
void filter1_predict(filter1_t *f, double gx, double gy, double gz);
void filter1_update(filter1_t *f, double ax, double ay, double az);

#endif