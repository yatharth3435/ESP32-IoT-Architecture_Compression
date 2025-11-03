#ifndef UART_H
#define UART_H
#include <stdint.h>

extern int64_t esp_time_mstest;

// Function prototypes
void uart_init(void);
void rx_task(void *arg);
void tx_uart(void *arg);

#endif
