#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <platform.h>
#include <esp_log.h>

#define TAG "main.c"

#define MAX_UART_BUFFER_SIZE 128

typedef void (*timer_callback_t)(void*);

typedef struct {
  uint8_t byte;
  uint8_t buffer[MAX_UART_BUFFER_SIZE];
  uint8_t header[2];
  char stage;
  uint16_t payload_size;
  int buffer_idx;
} uart_rx_t;

TaskHandle_t task_hangle_1 = NULL;
TaskHandle_t task_hangle_2 = NULL;

void platform_toggle_led(char led) {

}

void platform_delay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t platform_time_ms(void) {
  return esp_timer_get_time();
}

char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data) {
  return PLATFORM_OK;
}

char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data) {
  return PLATFORM_OK;
}

char platform_i2c_write_read_dma(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size,
    uint8_t *output, uint16_t output_size) {
  return PLATFORM_OK;
}

char platform_i2c_write_read(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size,
    uint8_t *output, uint16_t output_size, uint32_t timeout) {
  return PLATFORM_OK;
}

char platform_i2c_read(i2c_port__t port, uint8_t address, uint8_t *output, uint16_t output_size) {
  return PLATFORM_OK;
}

char platform_i2c_write(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size) {
  return PLATFORM_OK;
}

char platform_spi_write(spi_port_t spi_port, uint8_t *input, uint8_t size) {
  return PLATFORM_OK;
}

char platform_spi_write_read(spi_port_t spi_port, 
  uint8_t *input, uint16_t input_size,
  uint8_t *output, uint16_t output_size) {
  return PLATFORM_OK;
}

char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t data_size) {
  return PLATFORM_OK;
}

char platform_pwm_init(pwm_port_t port) {
  return PLATFORM_OK;
}

char platform_pwm_send(pwm_port_t port, uint32_t data) {
  return PLATFORM_OK;
}

char platform_dshot_init(dshot_port_t port) {
  switch (port) {
  case DSHOT_PORT1:
    break;
  case DSHOT_PORT2:
    break;
  case DSHOT_PORT3:
    break;
  case DSHOT_PORT4:
    break;
  default:
    break;
  }

  return PLATFORM_OK;
}

char platform_dshot_send(dshot_port_t port, uint16_t data) {
  return PLATFORM_OK;
}

char platform_dshot_ex_init(dshot_ex_port_t port) {
  switch (port) {
  case DSHOT_EX_PORT1:
    break;
  case DSHOT_EX_PORT2:
    break;
  case DSHOT_EX_PORT3:
    break;
  case DSHOT_EX_PORT4:
    break;
  default:
    break;
  }

  return PLATFORM_OK;
}

char platform_dshot_ex_send(dshot_ex_port_t port, uint32_t data) {
  return PLATFORM_OK;
}

void platform_console(const char *format, ...) {
  va_list args;
  va_start(args, format);
  esp_log_writev(ESP_LOG_INFO, "", format, args);
  va_end(args);
}

static void create_timer(timer_callback_t callback, uint64_t freq) {
  const esp_timer_create_args_t timer_args = {
    .callback = callback,
    .name = "Timer"
  };
  esp_timer_handle_t timer_handler;
  esp_timer_create(&timer_args, &timer_handler);
  esp_timer_start_periodic(timer_handler, 1000000/freq);
}

void core0() {
  while (1) { platform_delay(1000); }
}

void core1() {
  // Setup I2Cs

  // Setup SPIs

  // Setup UARTs

  // Setup PWMs

  // Setup DSHOT

  // Setup timers
  create_timer(platform_scheduler_4khz, 4000);
  create_timer(platform_scheduler_2khz, 2000);
  create_timer(platform_scheduler_1khz, 1000);
  create_timer(platform_scheduler_500hz, 500);
  create_timer(platform_scheduler_250hz, 250);
  create_timer(platform_scheduler_100hz, 100);
  create_timer(platform_scheduler_50hz, 50);
  create_timer(platform_scheduler_25hz, 25);
  create_timer(platform_scheduler_10hz, 10);
  create_timer(platform_scheduler_5hz, 5);
  create_timer(platform_scheduler_1hz, 1);

  // Setup platform modules
  platform_setup();

  while (1) {
    platform_loop();
    platform_delay(10);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Start program");

  xTaskCreatePinnedToCore(core0, "Core 0", 4096, NULL, 2, &task_hangle_1, 0);
  xTaskCreatePinnedToCore(core1, "Core 1", 4096, NULL, 1, &task_hangle_2, 1);

  while (1) { platform_delay(1000); }
}
