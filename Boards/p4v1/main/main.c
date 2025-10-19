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

static void toggle_led(char led) {

}

static void delay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static uint32_t time_ms(void) {
  return esp_timer_get_time();
}

static char storage_read(uint16_t start, uint16_t size, uint8_t *data) {
  return 0;
}

static char storage_write(uint16_t start, uint16_t size, uint8_t *data) {
  return 0;
}

static char i2c_write_read_dma(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size,
    uint8_t *output, uint16_t output_size) {
  return ESP_OK;;
}

static char i2c_write_read(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size,
    uint8_t *output, uint16_t output_size, uint32_t timeout) {
  return ESP_OK;
}

static char i2c_read(i2c_port_t port, uint8_t address, uint8_t *output, uint16_t output_size) {
  return ESP_OK;
}

static char i2c_write(i2c_port_t port, uint8_t address, uint8_t *input, uint16_t input_size) {
  return ESP_OK;
}

static char uart_send(uart_port_t port, uint8_t *data, uint16_t data_size) {
  return ESP_OK;
}

static char pwm_init(pwm_port_t port) {
  return 0;
}

static char pwm_send(pwm_port_t port, uint32_t data) {
  return 0;
}

static char _dshot_init(dshot_port_t port) {
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

  return 0;
}

static char dshot_send(dshot_port_t port, uint16_t data) {
  return 0;
}

static char _dshot_ex_init(dshot_ex_port_t port) {
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

  return 0;
}

static char dshot_ex_send(dshot_ex_port_t port, uint32_t data) {
  return 0;
}

static void handle_db_msg(uart_rx_t *msg) {
  if ((msg->header[0] == 'd' && msg->header[1] == 'b')) { // DB message
    platform_receive_internal_message(msg->buffer, msg->payload_size);
  }
}

static void timer_4khz(void*) { platform_scheduler_4khz(); }
static void timer_2khz(void*) { platform_scheduler_2khz(); }
static void timer_1khz(void*) { platform_scheduler_1khz(); }
static void timer_500hz(void*) { platform_scheduler_500hz(); }
static void timer_250hz(void*) { platform_scheduler_250hz(); }
static void timer_100hz(void*) { platform_scheduler_100hz(); }
static void timer_50hz(void*) { platform_scheduler_50hz(); }
static void timer_25hz(void*) { platform_scheduler_25hz(); }
static void timer_10hz(void*) { platform_scheduler_10hz(); }
static void timer_5hz(void*) { platform_scheduler_5hz(); }
static void timer_1hz(void*) { platform_scheduler_1hz(); }

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
  create_timer(timer_4khz, 4000);
  create_timer(timer_2khz, 2000);
  create_timer(timer_1khz, 1000);
  create_timer(timer_500hz, 500);
  create_timer(timer_250hz, 250);
  create_timer(timer_100hz, 100);
  create_timer(timer_50hz, 50);
  create_timer(timer_25hz, 25);
  create_timer(timer_10hz, 10);
  create_timer(timer_5hz, 5);
  create_timer(timer_1hz, 1);

  while (1) { delay(1000); }
}

void core1() {
  while (1) { delay(1000); }
}

void app_main(void) {
  ESP_LOGI(TAG, "Start program");

  // Register platform functions
  platform_register_toggle_led(toggle_led);
  platform_register_time_ms(time_ms);
  platform_register_delay(delay);
  platform_register_storage(storage_read, storage_write);
  platform_register_io_functions(
    i2c_write_read_dma,
    i2c_write_read,
    i2c_read, i2c_write,
    uart_send,
    pwm_init,
    pwm_send,
    _dshot_init,
    dshot_send,
    _dshot_ex_init,
    dshot_ex_send);

  // Setup platform modules
  platform_setup();

  xTaskCreatePinnedToCore(core0, "Core 0", 4096, NULL, 1, &task_hangle_1, 0);
  xTaskCreatePinnedToCore(core1, "Core 1", 4096, NULL, 1, &task_hangle_2, 1);

  while (1) {
    platform_loop();
    delay(10);
  }
}
