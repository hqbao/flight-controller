#include <pubsub.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct function_list {
	subscriber_callback_t callback;
	struct function_list *next;
} function_list_t;

static function_list_t* g_function_pointers[TOPIC_NULL];

void publish(topic_t topic, uint8_t *data, size_t size) {
	function_list_t *cur_fn = g_function_pointers[topic];
	while (cur_fn != NULL) {
		if (cur_fn->callback != NULL) {
			cur_fn->callback(data, size);
		}
		cur_fn = cur_fn->next;
	}
}

void subscribe(topic_t topic, subscriber_callback_t callback) {
	function_list_t *fn = (function_list_t*) malloc(sizeof(function_list_t));
	fn->callback = callback;
	fn->next = NULL;

	if (g_function_pointers[topic] == NULL) {
		printf("Topic %d has the first function\n", topic);
		g_function_pointers[topic] = fn;
	}
	else {
		function_list_t *cur_fn = g_function_pointers[topic];
		int fn_count = 2;
		while (cur_fn->next != NULL) {
			cur_fn = cur_fn->next;
			fn_count += 1;
		}

		cur_fn->next = fn;
		printf("Topic %d has %d functions\n", topic, fn_count);
	}
}
