#ifndef CONFIG_INTERNAL_H
#define CONFIG_INTERNAL_H

#include <pubsub.h>
#include <messages.h>
#include <string.h>

/* Helper: request a range of param IDs from local_storage */
void config_request_params(param_id_e first, param_id_e last);

/* --- Gyro --- */
void config_gyro_on_result(param_storage_t *p);
void config_gyro_on_save(param_storage_t *p);
void config_gyro_publish(void);
void config_gyro_request_load(void);
void config_gyro_on_request(uint8_t *data, size_t size);

/* --- Accel --- */
void config_accel_on_result(param_storage_t *p);
void config_accel_on_save(param_storage_t *p);
void config_accel_publish(void);
void config_accel_request_load(void);
void config_accel_on_request(uint8_t *data, size_t size);

/* --- Mag --- */
void config_mag_on_result(param_storage_t *p);
void config_mag_on_save(param_storage_t *p);
void config_mag_publish(void);
void config_mag_request_load(void);
void config_mag_on_request(uint8_t *data, size_t size);

/* --- Tuning --- */
void config_tuning_on_result(param_storage_t *p);
void config_tuning_on_save(param_storage_t *p);
void config_tuning_publish(void);
void config_tuning_request_load(void);
void config_tuning_on_db_message(uint8_t *data, size_t size);

#endif
