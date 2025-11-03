#ifndef POST_DATA_H
#define POST_DATA_H

#include <stddef.h>
#include <stdint.h>

void send_packet_via_http(const uint8_t *data, size_t len);
void http_post_task(void *arg);

#endif