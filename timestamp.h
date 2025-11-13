#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ring_buffer.h"

extern int64_t first_packet_systime;

// RT Buffer - FreeRTOS Queue (created in main.c)
extern QueueHandle_t rt_buffer;
#define RT_BUFFER_SIZE 50 

// NTP sync tracking variables
extern int64_t first_synced_epoch;
extern int64_t first_synced_esp_ms;
extern volatile bool ntp_synced;     // Flag to indicate NTP sync status
extern volatile bool backlog_complete; // Flag to indicate backlog processing is done

extern uint32_t send_data_timeout;
extern int64_t cmp_fill_start;

// Timing functions for NTP and WiFi
void time_init(void);

// Timestamp task - handles backward and forward timestamping
void timestamp_task(void *param);

#endif
