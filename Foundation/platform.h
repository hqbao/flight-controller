#ifndef PLATFORM_H
#define PLATFORM_H

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

typedef enum {
	I2C_PORT1 = 0,
	I2C_PORT2,
	I2C_PORT3,
	I2C_PORT4,
} i2c_port_t;

typedef enum {
	SPI_PORT1 = 0,
	SPI_PORT2,
	SPI_PORT3,
	SPI_PORT4,
} spi_port_t;

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

// Outcoming functions
char platform_i2c_write_read_dma(i2c_port_t port, uint8_t address, 
	uint8_t *input, uint16_t input_size,
	uint8_t *output, uint16_t output_size);
char platform_i2c_write_read(i2c_port_t port, uint8_t address, 
	uint8_t *input, uint16_t input_size,
	uint8_t *output, uint16_t output_size, uint32_t timeout);
char platform_i2c_read(i2c_port_t port, uint8_t address, 
	uint8_t *output, uint16_t output_size);
char platform_i2c_write(i2c_port_t port, uint8_t address, 
	uint8_t *input, uint16_t input_size);

char platform_spi_write(spi_port_t spi_port, uint8_t *input, uint8_t size);
char platform_spi_write_read(spi_port_t spi_port, 
  uint8_t *input, uint16_t input_size,
  uint8_t *output, uint16_t output_size);

char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t data_size);

char platform_pwm_init(pwm_port_t port);
char platform_pwm_send(pwm_port_t port, uint32_t data);

char platform_dshot_init(dshot_port_t port);
char platform_dshot_send(dshot_port_t port, uint16_t data);

char platform_dshot_ex_init(dshot_ex_port_t port);
char platform_dshot_ex_send(dshot_ex_port_t port, uint32_t data);

void platform_toggle_led(char led);
uint32_t platform_time_ms(void);
void platform_delay(uint32_t ms);

char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data);
char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data);

void platform_console(const char *format, ...);

// Incoming functions
void platform_scheduler_1hz(void*);
void platform_scheduler_5hz(void*);
void platform_scheduler_10hz(void*);
void platform_scheduler_25hz(void*);
void platform_scheduler_50hz(void*);
void platform_scheduler_100hz(void*);
void platform_scheduler_250hz(void*);
void platform_scheduler_500hz(void*);
void platform_scheduler_1khz(void*);
void platform_scheduler_2khz(void*);
void platform_scheduler_4khz(void*);
void platform_scheduler_8khz(void*);

void platform_receive_internal_message(uint8_t *data, uint16_t size);
void platform_receive_external_message(uint8_t *data, uint16_t size);
void platform_on_fault_detected(uint8_t *data, uint16_t size);

void platform_setup(void);
void platform_loop(void);

void platform_i2c_data_dma_callback(i2c_port_t port);
void platform_spi_data_dma_callback(spi_port_t port);
void platform_uart_data_dma_callback(uart_port_t port);

#define print platform_console

#endif
