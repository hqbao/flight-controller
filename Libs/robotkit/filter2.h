#ifndef FILTER2_H
#define FILTER2_H

typedef struct {
    double pred_encoder;
    double gyro_freq;
    double k;
    char no_correction;
} filter2_t;

void filter2_init(filter2_t *f, double k, double freq);
void filter2_update_gyro1(filter2_t *f, double gyro);
void filter2_update_gyro2(filter2_t *f, double gyro);
void filter2_update_encoder(filter2_t *f, double encoder);

#endif