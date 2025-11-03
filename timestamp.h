#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern int64_t first_packet_systime;
 
// Type for packets with timestamp
typedef struct {
    uint8_t data[62];
    int data_len;
    int64_t sys_time;
} timestamped_packet_t;

// RT Buffer - FreeRTOS Queue (created in main.c)
extern QueueHandle_t rt_buffer;
#define RT_BUFFER_SIZE 50 

// CMP Buffer - Manual Circular Buffer
// Optimized size to reduce DRAM usage from 20kB to ~11kB
#define CMP_BUFFER_CAPACITY 250   // packet size: 88 * 250 = 22kB
#define CMP_BUFFER_LT 75         // Lower Threshold: 30% of capacity
#define CMP_BUFFER_UT 200         // Upper Threshold: 80% of capacity

typedef struct {
    timestamped_packet_t packets[CMP_BUFFER_CAPACITY];
    volatile uint32_t head;          // Write index (written by timestamp_task)
    volatile uint32_t tail;          // Read index (written by http_post_task)
    volatile uint32_t count;         // Number of packets currently in buffer
} cmp_buffer_t;

// CMP Buffer instance
extern cmp_buffer_t cmp_buffer;

// NTP sync tracking variables
extern int64_t first_synced_epoch;
extern int64_t first_synced_esp_ms;
extern volatile bool ntp_synced;     // Flag to indicate NTP sync status
extern volatile bool backlog_complete; // Flag to indicate backlog processing is done

// CMP Buffer Functions
void cmp_buffer_init(void);
bool cmp_buffer_push(const timestamped_packet_t *packet);
bool cmp_buffer_pop(timestamped_packet_t *packet);
uint32_t cmp_buffer_count(void);
bool cmp_buffer_is_empty(void);

// Timing functions for NTP and WiFi
void time_init(void);

// Timestamp task - handles backward and forward timestamping
void timestamp_task(void *param);

#endif
