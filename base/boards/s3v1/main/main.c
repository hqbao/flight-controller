#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <platform.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <esp_camera.h>
#include <optflow.h>
#include "image_util.h"

#define TAG "main.c"

typedef void (*timer_callback_t)(void*);

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38 // 38 or 2
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define WIDTH 64
#define HEIGHT 64

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))

typedef struct {
  int16_t dx;
  int16_t dy;
  int16_t dz;
  int64_t z_raw;
  int16_t z_alt;
} optflow_t;

static volatile char g_frame_captured = 0;
static volatile optflow_t g_optflow = {0, 0, 0, 0, 0};
static uint8_t g_frame[WIDTH*HEIGHT] = {0,};

static TaskHandle_t task_hangle_1 = NULL;
static TaskHandle_t task_hangle_2 = NULL;

static spi_device_handle_t spi1;
static spi_device_handle_t *spi_device_handlers[4] = {&spi1, NULL, NULL, NULL};

static i2c_master_dev_handle_t i2c1;
static i2c_master_dev_handle_t *i2c_master_dev_handlers[4] = {&i2c1, NULL, NULL, NULL};

static esp_err_t i2c_init(void) {
  // I2C bus configuration
  i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_5,
    .scl_io_num = GPIO_NUM_6,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };
  
  // Initialize I2C master bus
  i2c_master_bus_handle_t bus_handle;
  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C master bus: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Add ICM42688P as a device on the bus
  i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68,
    .scl_speed_hz = 1000000,  // 400kHz
  };
  
  ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, i2c_master_dev_handlers[0]);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "I2C initialized");
  return ret;
}

static void spi_init(void) {    
  spi_bus_config_t buscfg = {
    .miso_io_num = GPIO_NUM_8,
    .mosi_io_num = GPIO_NUM_9,
    .sclk_io_num = GPIO_NUM_7,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
  };
  
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz
    .mode = 3,                          // SPI mode 3 (CPOL=1, CPHA=1)
    .spics_io_num = GPIO_NUM_1,
    .queue_size = 7,
    .command_bits = 0,                  // No command bits, we'll use full transactions
    .address_bits = 0,                  // No address bits
    .flags = 0,                         // Full duplex mode
  };
  
  // Initialize SPI bus
  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  
  // Attach device to SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, spi_device_handlers[0]);
  ESP_ERROR_CHECK(ret);
  
  ESP_LOGI(TAG, "SPI initialized");
}

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
  esp_err_t ret = i2c_master_transmit_receive(*i2c_master_dev_handlers[port], 
    input, input_size, output, output_size, -1);
  if (ret == ESP_OK) platform_i2c_data_dma_callback(port);
  return ret == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_i2c_write_read(i2c_port__t port, uint8_t address, 
  uint8_t *input, uint16_t input_size,
  uint8_t *output, uint16_t output_size, uint32_t timeout) {
  esp_err_t ret = i2c_master_transmit_receive(*i2c_master_dev_handlers[port], 
    input, input_size, output, output_size, -1);
  return ret == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_i2c_read(i2c_port__t port, uint8_t address, uint8_t *output, uint16_t output_size) {
  return PLATFORM_NOT_SUPPORT;
}

char platform_i2c_write(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size) {
  esp_err_t ret = i2c_master_transmit(*i2c_master_dev_handlers[port], input, input_size, -1);
  return ret == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_spi_write(spi_port_t spi_port, uint8_t *input, uint8_t size) {
  spi_transaction_t t = {
    .length = size * 8, // In bits
    .tx_buffer = input,
    .rx_buffer = NULL,
  };
  esp_err_t status = spi_device_polling_transmit(*spi_device_handlers[spi_port], &t);
  return status == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_spi_write_read(spi_port_t spi_port, 
  uint8_t *input, uint16_t input_size,
  uint8_t *output, uint16_t output_size) {
  spi_transaction_t t = {
    .length = (input_size + output_size) * 8,  // Input/Output size in bits
    .tx_buffer = input,
    .rx_buffer = output,
  };
  
  esp_err_t status = spi_device_polling_transmit(*spi_device_handlers[spi_port], &t);
  platform_spi_data_dma_callback(spi_port);
  return status == ESP_OK ? PLATFORM_OK : PLATFORM_ERROR;
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

static void init_cam(void) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 1;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "Camera init failed with error 0x%x\n", err);
    return;
  }
}

static void frame_timer(void *param) {
  // Capture frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGI(TAG, "Capture failed\n");
    return;
  }

  // ESP_LOGI(TAG, "%dx%d, %d, %d\n", fb->height, fb->width, fb->format, fb->len);
  esp_camera_fb_return(fb);

  fast_crop_and_resize_bilinear(
    fb->buf, fb->width, fb->height,
    g_frame, WIDTH, HEIGHT,
    (int)((fb->width - fb->height) * 0.5), 0, fb->height, fb->height);

  g_frame_captured = 1;
}

static void calc_otpflw(void) {
  g_frame_captured = 0;

  float clearity = 0;
  float dx_mm = 0;
  float dy_mm = 0;
  float rotation = 0;
  int mode = 0;
  optflow_calc(g_frame, &dx_mm, &dy_mm, &rotation, &clearity, &mode);

  // Output is in mm/frame
  dx_mm = LIMIT(dx_mm, -100, 100);
  dy_mm = LIMIT(-dy_mm, -100, 100);
  int dx_int = dx_mm;
  int dy_int = dy_mm;

  static int64_t t_prev = 0;
  int64_t t1 = esp_timer_get_time();
  int dt_actual = t1 - t_prev;
  t_prev = t1;
  int f_actual = 1000000/dt_actual;

  ESP_LOGI(TAG, "$%d\t%d\t%d\t%d\t%f\t%d\t",
    (int)(dx_int), (int)(dy_int), (int)(g_optflow.dz), 
    (int)(g_optflow.z_raw), clearity, f_actual);
}

void core1() {
  // Init camera
  init_cam();

  // Init optical flow
  optflow_init(WIDTH, HEIGHT, 1);  // 1=hybrid mode, 0=dense only

  while (1) {
    frame_timer(NULL);
    calc_otpflw();
  }
}

void core0() {
  // Setup I2Cs
  i2c_init();

  // Setup SPIs
  // spi_init();

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

  xTaskCreatePinnedToCore(core0, "Core 0", 4096, NULL, 10, &task_hangle_1, 0);
  xTaskCreatePinnedToCore(core1, "Core 1", 4096, NULL, 10, &task_hangle_2, 1);

  while (1) { platform_delay(1000); }
}
