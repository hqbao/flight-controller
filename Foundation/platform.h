#ifndef PLATFORM_H
#define PLATFORM_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
	I2C_PORT1 = 0,
	I2C_PORT2,
	I2C_PORT3,
	I2C_PORT4,
} i2c_port_t;

typedef enum {
	UART_PORT1 = 0,
	UART_PORT2,
	UART_PORT3,
	UART_PORT4,
} uart_port_t;

typedef enum {
	PWM_PORT1 = 0,
	PWM_PORT2,
	PWM_PORT3,
	PWM_PORT4,
} pwm_port_t;

typedef enum {
	DSHOT_PORT1 = 0,
	DSHOT_PORT2,
	DSHOT_PORT3,
	DSHOT_PORT4,
} dshot_port_t;

typedef enum {
	DSHOT_EX_PORT1 = 0,
	DSHOT_EX_PORT2,
	DSHOT_EX_PORT3,
	DSHOT_EX_PORT4,
} dshot_ex_port_t;

typedef char (*i2c_write_read_dma_t)(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size,
		uint8_t *output, uint16_t output_size);
typedef char (*i2c_write_read_t)(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size,
		uint8_t *output, uint16_t output_size, uint32_t timeout);
typedef char (*i2c_read_t)(i2c_port_t port, uint8_t address, uint8_t *output, uint16_t output_size);
typedef char (*i2c_write_t)(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size);

typedef char (*uart_send_t)(uart_port_t port, uint8_t *data, uint16_t data_size);

typedef char (*pwm_init_t)(pwm_port_t port);
typedef char (*pwm_send_t)(pwm_port_t port, uint32_t data);

typedef char (*dshot_init_t)(dshot_port_t port);
typedef char (*dshot_send_t)(dshot_port_t port, uint16_t data);

typedef char (*dshot_ex_init_t)(dshot_ex_port_t port);
typedef char (*dshot_ex_send_t)(dshot_ex_port_t port, uint32_t data);

typedef void (*toggle_led_t)(char led);
typedef uint32_t (*time_ms_t)(void);
typedef void (*delay_t)(uint32_t ms);

typedef char (*storage_read_t)(uint16_t start, uint16_t size, uint8_t *data);
typedef char (*storage_write_t)(uint16_t start, uint16_t size, uint8_t *data);

typedef struct {
	i2c_write_read_dma_t i2c_write_read_dma;
	i2c_write_read_t i2c_write_read;
	i2c_read_t i2c_read;
	i2c_write_t i2c_write;
	uart_send_t uart_send;
	pwm_init_t pwm_init;
	pwm_send_t pwm_send;
	dshot_init_t dshot_init;
	dshot_send_t dshot_send;
	dshot_ex_init_t dshot_ex_init;
	dshot_ex_send_t dshot_ex_send;
} serial_port_t;

typedef struct {
	serial_port_t port;
	toggle_led_t toggle_led;
	time_ms_t time_ms;
	delay_t delay;
	storage_read_t storage_read;
	storage_write_t storage_write;
} platform_t;

void platform_register_io_functions(
	i2c_write_read_dma_t i2c_write_read_dma,
	i2c_write_read_t i2c_write_read,
	i2c_read_t i2c_read,
	i2c_write_t i2c_write,
	uart_send_t uart_send,
	pwm_init_t pwm_init,
	pwm_send_t pwm_send,
	dshot_init_t dshot_init,
	dshot_send_t dshot_send,
	dshot_ex_init_t dshot_ex_init,
	dshot_ex_send_t dshot_ex_send);

void platform_register_toggle_led(toggle_led_t toggle_led);

void platform_register_time_ms(time_ms_t time_ms);

void platform_register_delay(delay_t delay);

void platform_register_storage(storage_read_t storage_read, storage_write_t storage_write);

platform_t* get_platform(void);

void platform_scheduler_1hz(void);
void platform_scheduler_5hz(void);
void platform_scheduler_10hz(void);
void platform_scheduler_25hz(void);
void platform_scheduler_50hz(void);
void platform_scheduler_100hz(void);
void platform_scheduler_250hz(void);
void platform_scheduler_500hz(void);
void platform_scheduler_1khz(void);
void platform_scheduler_2khz(void);
void platform_scheduler_4khz(void);
void platform_scheduler_8khz(void);

void platform_receive_internal_message(uint8_t *data, uint16_t size);
void platform_receive_external_message(uint8_t *data, uint16_t size);
void platform_on_fault_detected(uint8_t *data, uint16_t size);

void platform_setup(void);
void platform_loop(void);

#define platform_i2c_write_read_dma get_platform()->port.i2c_write_read_dma
#define platform_i2c_write_read 	get_platform()->port.i2c_write_read
#define platform_i2c_read 			get_platform()->port.i2c_read
#define platform_i2c_write 			get_platform()->port.i2c_write

#define platform_uart_send 			get_platform()->port.uart_send

#define platform_pwm_init 			get_platform()->port.pwm_init
#define platform_pwm_send 			get_platform()->port.pwm_send

#define platform_dshot_init 		get_platform()->port.dshot_init
#define platform_dshot_send 		get_platform()->port.dshot_send

#define platform_dshot_ex_init 		get_platform()->port.dshot_ex_init
#define platform_dshot_ex_send 		get_platform()->port.dshot_ex_send

#define platform_toggle_led 		get_platform()->toggle_led
#define platform_time_ms 			get_platform()->time_ms
#define platform_delay 				get_platform()->delay

#define platform_storage_read		get_platform()->storage_read
#define platform_storage_write		get_platform()->storage_write

#endif
