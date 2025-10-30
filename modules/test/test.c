#include "test.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

static int g_test_counters[10];

static void loop_4khz(uint8_t *data, size_t size) { g_test_counters[0]++; }
static void loop_2khz(uint8_t *data, size_t size) { g_test_counters[1]++; }
static void loop_1khz(uint8_t *data, size_t size) { g_test_counters[2]++; }
static void loop_500hz(uint8_t *data, size_t size) { g_test_counters[3]++; }
static void loop_250hz(uint8_t *data, size_t size) { g_test_counters[4]++; }
static void loop_100hz(uint8_t *data, size_t size) { g_test_counters[5]++; }
static void loop_50hz(uint8_t *data, size_t size) { g_test_counters[6]++; }
static void loop_25hz(uint8_t *data, size_t size) { g_test_counters[7]++; }
static void loop_10hz(uint8_t *data, size_t size) { g_test_counters[8]++; }
static void loop_5hz(uint8_t *data, size_t size) { g_test_counters[9]++; }

static void loop_1hz(uint8_t *data, size_t size) {
  for (int i = 0; i < 10; i++) {
    int counter = g_test_counters[i];
    g_test_counters[i] = 0;
    print("counter[%d]: %d\n", i, counter);
  }
}

void test_setup(void) {
  print("test_setup\n");
  memset(g_test_counters, 0, sizeof(int) * 10);
  subscribe(SCHEDULER_4KHZ, loop_4khz);
  subscribe(SCHEDULER_2KHZ, loop_2khz);
  subscribe(SCHEDULER_1KHZ, loop_1khz);
  subscribe(SCHEDULER_500HZ, loop_500hz);
  subscribe(SCHEDULER_250HZ, loop_250hz);
  subscribe(SCHEDULER_100HZ, loop_100hz);
  subscribe(SCHEDULER_50HZ, loop_50hz);
  subscribe(SCHEDULER_25HZ, loop_25hz);
  subscribe(SCHEDULER_10HZ, loop_10hz);
  subscribe(SCHEDULER_5HZ, loop_5hz);
  subscribe(SCHEDULER_1HZ, loop_1hz);  
}
