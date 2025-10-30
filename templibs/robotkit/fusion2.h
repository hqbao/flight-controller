#ifndef FUSION2_H
#define FUSION2_H

typedef struct {
    double pred_encoder;
    double gyro_freq;
    double k;
    char no_correction;
} fusion2_t;

void fusion2_init(fusion2_t *f, double k, double freq);
void fusion2_update_gyro1(fusion2_t *f, double gyro);
void fusion2_update_gyro2(fusion2_t *f, double gyro);
void fusion2_update_encoder(fusion2_t *f, double encoder);

#endif