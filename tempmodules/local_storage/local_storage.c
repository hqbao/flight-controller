#include "local_storage.h"
#include <pubsub.h>
#include <platform.h>
#include <string.h>

#define LOCAL_STORAGE_SIZE 128

static uint8_t g_local_data[LOCAL_STORAGE_SIZE + 4] = {0}; // data + 4-byte checksum

static void local_storage_save(uint8_t *data, size_t size) {
	memcpy(g_local_data, data, LOCAL_STORAGE_SIZE);
}

static void local_storage_load(uint8_t *data, size_t size) {
	publish(LOCAL_STORAGE_RESULT, g_local_data, LOCAL_STORAGE_SIZE);
}

static void load_local_data(void) {

}

void local_storage_setup(void) {
	// Load flash memory data once
	load_local_data();

	subscribe(LOCAL_STORAGE_SAVE, local_storage_save);
	subscribe(LOCAL_STORAGE_LOAD, local_storage_load);
}
