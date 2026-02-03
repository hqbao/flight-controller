#include "noise_meas.h"
#include <pubsub.h>
#include <math.h>
#include <string.h>

// Define structure for stats accumulation
typedef struct {
    double sum_x;
    double sum_y;
    double sum_z;
    double sq_sum_x;
    double sq_sum_y;
    double sq_sum_z;
    uint32_t count;
    
    // Results
    float mean[3];
    float stddev[3];
} noise_stats_t;

static noise_stats_t g_gyro_stats = {0};
static noise_stats_t g_accel_stats = {0};

// Helper to update stats
static void update_stats(noise_stats_t *stats, float *data) {
    stats->sum_x += data[0];
    stats->sum_y += data[1];
    stats->sum_z += data[2];
    
    stats->sq_sum_x += data[0] * data[0];
    stats->sq_sum_y += data[1] * data[1];
    stats->sq_sum_z += data[2] * data[2];
    
    stats->count++;
}

// Helper to calculate final results
static void calculate_results(noise_stats_t *stats) {
    if (stats->count == 0) return;
    
    double n = (double)stats->count;
    
    // Mean
    stats->mean[0] = (float)(stats->sum_x / n);
    stats->mean[1] = (float)(stats->sum_y / n);
    stats->mean[2] = (float)(stats->sum_z / n);
    
    // Variance = E[X^2] - (E[X])^2
    double cx = stats->sq_sum_x / n;
    double cy = stats->sq_sum_y / n;
    double cz = stats->sq_sum_z / n;
    
    double mx = stats->mean[0];
    double my = stats->mean[1];
    double mz = stats->mean[2];

    double var_x = cx - (mx * mx);
    double var_y = cy - (my * my);
    double var_z = cz - (mz * mz);

    // StdDev
    stats->stddev[0] = (float)sqrt(var_x > 0 ? var_x : 0);
    stats->stddev[1] = (float)sqrt(var_y > 0 ? var_y : 0);
    stats->stddev[2] = (float)sqrt(var_z > 0 ? var_z : 0);
    
    // Reset accumulators
    stats->sum_x = 0;
    stats->sum_y = 0;
    stats->sum_z = 0;
    stats->sq_sum_x = 0;
    stats->sq_sum_y = 0;
    stats->sq_sum_z = 0;
    stats->count = 0;
}

// Called at high frequency (1kHz)
static void on_gyro_update(uint8_t *data, size_t size) {
    if (size < 3 * sizeof(float)) return;
    update_stats(&g_gyro_stats, (float*)data);
}

static void on_accel_update(uint8_t *data, size_t size) {
    if (size < 3 * sizeof(float)) return;
    update_stats(&g_accel_stats, (float*)data);
}

// Called at low frequency (5Hz) to report stats
static void report_noise(uint8_t *data, size_t size) {
    // Process Gyro
    calculate_results(&g_gyro_stats);
    // Process Accel
    calculate_results(&g_accel_stats);
    
    // Prepare message: 
    // [Gx_Mean, Gy_Mean, Gz_Mean, Gx_Std, Gy_Std, Gz_Std, 
    //  Ax_Mean, Ay_Mean, Az_Mean, Ax_Std, Ay_Std, Az_Std]
    // 12 floats = 48 bytes
    float msg[12];
    
    // Gyro
    msg[0] = g_gyro_stats.mean[0];
    msg[1] = g_gyro_stats.mean[1];
    msg[2] = g_gyro_stats.mean[2];
    msg[3] = g_gyro_stats.stddev[0];
    msg[4] = g_gyro_stats.stddev[1];
    msg[5] = g_gyro_stats.stddev[2];
    
    // Accel
    msg[6] = g_accel_stats.mean[0];
    msg[7] = g_accel_stats.mean[1];
    msg[8] = g_accel_stats.mean[2];
    msg[9] = g_accel_stats.stddev[0];
    msg[10] = g_accel_stats.stddev[1];
    msg[11] = g_accel_stats.stddev[2];
    
    publish(MONITOR_DATA, (uint8_t*)msg, sizeof(msg));
}

void noise_meas_setup(void) {
    subscribe(SENSOR_IMU1_GYRO_UPDATE, on_gyro_update);
    // Subscribe to accel update (published by imu.c)
    subscribe(SENSOR_IMU1_ACCEL_UPDATE, on_accel_update);
    
    subscribe(SCHEDULER_5HZ, report_noise); // 5Hz reporting
}
