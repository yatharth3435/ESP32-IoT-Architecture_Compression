#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>


// Forward declaration of timestamped_packet_t (actual definition in timestamp.h)
typedef struct {
    uint8_t data[62];
    int64_t sys_time;
    int data_len;
} timestamped_packet_t;

// CMP Buffer Configuration
#define CMP_BUFFER_CAPACITY 250
#define CMP_BUFFER_LT 75
#define CMP_BUFFER_UT 200

// CMP Buffer Structure
typedef struct {
    timestamped_packet_t packets[CMP_BUFFER_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} cmp_buffer_t;

// CMP Buffer instance (defined in ring_buffer.c)
extern cmp_buffer_t cmp_buffer;

// CMP Buffer Functions
void cmp_buffer_init(void);
bool cmp_buffer_push(const timestamped_packet_t *packet);
bool cmp_buffer_pop(timestamped_packet_t *packet);
uint32_t cmp_buffer_count(void);
bool cmp_buffer_is_empty(void);



// Sendoff Buffer Configuration
#define SENDOFF_BUFFER_CAPACITY 7168  // 7kB
#define SENDOFF_BUFFER_LT 5018        // 70% of 7168
#define SENDOFF_BUFFER_UT 6093        // 85% of 7168

// Sendoff Buffer Structure
typedef struct {
    uint8_t data[SENDOFF_BUFFER_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} sendoff_buffer_t;

// Sendoff Buffer instance (defined in ring_buffer.c)
extern sendoff_buffer_t sendoff_buffer;

// Sendoff Buffer Functions
void sendoff_buffer_init(void);
bool sendoff_buffer_push(const uint8_t *data, uint32_t len);
bool sendoff_buffer_pop(uint8_t *data, uint32_t len);
uint32_t sendoff_buffer_count(void);
bool sendoff_buffer_is_empty(void);
uint32_t sendoff_buffer_peek(uint8_t *data, uint32_t max_len);

#endif // RING_BUFFER_H