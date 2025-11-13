#include "ring_buffer.h"
#include <string.h>
#include "esp_log.h"

// CMP BUFFER IMPLEMENTATION

// CMP Buffer instance
cmp_buffer_t cmp_buffer;

//Set up the circular buffer structure with head=0, tail=0, count=0
void cmp_buffer_init(void) {
    cmp_buffer.head = 0;
    cmp_buffer.tail = 0;

    //stores current pointer location
    cmp_buffer.count = 0;
    
    //ESP_LOGI("CMP_BUFFER", "Initialized with capacity=%d, LT=%d, UT=%d", CMP_BUFFER_CAPACITY, CMP_BUFFER_LT, CMP_BUFFER_UT);
}

/*
This function pushes a packet into the CMP buffer, input a pointer to the packet to be pushed
Returns false if buffer is full. Only timestamp_task writes to this buffer
*/
bool cmp_buffer_push(const timestamped_packet_t *packet) {

    // Check if buffer is full
    if (cmp_buffer.count >= CMP_BUFFER_CAPACITY) {
        ESP_LOGW("CMP_BUFFER", "Buffer full! Dropping packet. Count=%lu", cmp_buffer.count);
        return false;
    }
    
    // Copy packet to buffer at head position
    memcpy(&cmp_buffer.packets[cmp_buffer.head], packet, sizeof(timestamped_packet_t));
    
    // Update head pointer (circular) and current count pointer
    cmp_buffer.head = (cmp_buffer.head + 1) % CMP_BUFFER_CAPACITY;
    cmp_buffer.count++;
    
    return true;
}

/*
This function pops a packet from the CMP buffer, input a pointer to the variable storing the popped data
Returns false if buffer is empty. Only http_post reads from this buffer
*/
bool cmp_buffer_pop(timestamped_packet_t *packet) {

    // Check if buffer is empty
    if (cmp_buffer.count == 0) return false;
    
    // Copy packet from buffer at tail position
    memcpy(packet, &cmp_buffer.packets[cmp_buffer.tail], sizeof(timestamped_packet_t));
    
    // Update tail pointer (circular) and decrement current count pointer
    cmp_buffer.tail = (cmp_buffer.tail + 1) % CMP_BUFFER_CAPACITY;
    cmp_buffer.count--;
    
    return true;
}


//Get the current count of packets in buffer, returns the count variable
uint32_t cmp_buffer_count(void) {
    return cmp_buffer.count;
}


//Check if CMP buffer is empty. true if empty, false otherwise
bool cmp_buffer_is_empty(void) {
    return (cmp_buffer_count() == 0);
}


// ============================================================================
// SENDOFF BUFFER IMPLEMENTATION
// ============================================================================

// Sendoff Buffer instance
sendoff_buffer_t sendoff_buffer;

/*
Initialize Sendoff buffer with head=0, tail=0, count=0
*/
void sendoff_buffer_init(void) {
    sendoff_buffer.head = 0;
    sendoff_buffer.tail = 0;
    sendoff_buffer.count = 0;
    
    ESP_LOGI("SENDOFF_BUFFER", "Initialized with capacity=%d bytes, LT=%d bytes, UT=%d bytes", SENDOFF_BUFFER_CAPACITY, SENDOFF_BUFFER_LT, SENDOFF_BUFFER_UT);
}

/*
Push data into Sendoff buffer
Parameters:
  - data: pointer to data to push
  - len: length of data in bytes
Returns false if buffer doesn't have enough space
*/
bool sendoff_buffer_push(const uint8_t *data, uint32_t len) {

    // Check if buffer has enough space
    if (sendoff_buffer.count + len > SENDOFF_BUFFER_CAPACITY) {
        ESP_LOGW("SENDOFF_BUFFER", "Buffer full! Cannot push %lu bytes. Current count=%lu", len, sendoff_buffer.count);
        return false;
    }
    
    // Copy data byte-by-byte in circular fashion
    for (uint32_t i = 0; i < len; i++) {
        sendoff_buffer.data[sendoff_buffer.head] = data[i];
        sendoff_buffer.head = (sendoff_buffer.head + 1) % SENDOFF_BUFFER_CAPACITY;
    }
    
    sendoff_buffer.count += len;
    
    return true;
}

/*
Pop data from Sendoff buffer
Parameters:
  - data: pointer to buffer to store popped data
  - len: number of bytes to pop
Returns false if buffer doesn't have enough data
*/
bool sendoff_buffer_pop(uint8_t *data, uint32_t len) {

    // Check if buffer has enough data
    if (sendoff_buffer.count < len) {
        ESP_LOGW("SENDOFF_BUFFER", "Not enough data! Requested %lu bytes, available %lu bytes", len, sendoff_buffer.count);
        return false;
    }
    
    // Copy data byte-by-byte in circular fashion
    for (uint32_t i = 0; i < len; i++) {
        data[i] = sendoff_buffer.data[sendoff_buffer.tail];
        sendoff_buffer.tail = (sendoff_buffer.tail + 1) % SENDOFF_BUFFER_CAPACITY;
    }
    
    sendoff_buffer.count -= len;
    
    return true;
}

/*
Peek data from Sendoff buffer without removing it
Parameters:
  - data: pointer to buffer to store peeked data
  - max_len: maximum number of bytes to peek
Returns actual number of bytes peeked
*/
uint32_t sendoff_buffer_peek(uint8_t *data, uint32_t max_len) {

    // Determine how many bytes to peek
    uint32_t bytes_to_peek = (sendoff_buffer.count < max_len) ? sendoff_buffer.count : max_len;
    
    // Copy data without modifying tail pointer
    uint32_t temp_tail = sendoff_buffer.tail;
    for (uint32_t i = 0; i < bytes_to_peek; i++) {
        data[i] = sendoff_buffer.data[temp_tail];
        temp_tail = (temp_tail + 1) % SENDOFF_BUFFER_CAPACITY;
    }
    
    return bytes_to_peek;
}

/*
Get the current count of bytes in Sendoff buffer
*/
uint32_t sendoff_buffer_count(void) {
    return sendoff_buffer.count;
}

/*
Check if Sendoff buffer is empty
*/
bool sendoff_buffer_is_empty(void) {
    return (sendoff_buffer.count == 0);
}