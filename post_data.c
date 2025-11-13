#include "esp_http_client.h"
#include "timestamp.h"
#include "post_data.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


// Configuration for batching logic
// Reduced from 50 to 30 to save RAM (~3kB savings)
#define MAX_BATCH_PACKETS 75
#define BATCH_TIMEOUT_MS 1500

#define MODIFIED_PACKET_BUFFER_SIZE 90
#define MAX_PAYLOAD_SIZE (MODIFIED_PACKET_BUFFER_SIZE * MAX_BATCH_PACKETS)

//Function to send Base-64 encoded data to provided server via HTTP POST. Content-Type is set to "text/plain" for Base64 data.

void send_packet_via_http(const uint8_t *data, size_t len) {

    //set the http client configuration using given parameters, provide url of server to be used
    esp_http_client_config_t config = {
        .url = "http://192.168.1.101:5000/esp32/data",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };

    //set up an instance of esp http client based on config provided above 
    esp_http_client_handle_t client = esp_http_client_init(&config);

    //LOG
    if (!client) {
        ESP_LOGE("HTTP", "Failed to initialise HTTP client");
        return;
    }

    //Set http request header, this function must be called after esp_http_client_init and before any perform function.
    esp_http_client_set_header(client, "Content-Type", "text/plain");

    //set post data, note that the data passed to this function is a pointer and not a copy
    esp_http_client_set_post_field(client, (const char *)data, len);

    //perform all operations of esp_http_client
    esp_err_t err = esp_http_client_perform(client);

    //LOG
    if (err == ESP_OK) {
        ESP_LOGI("HTTP", "POST successful, sent %d bytes, status %d", len, esp_http_client_get_status_code(client));
    } 
    //LOG
    else ESP_LOGE("HTTP", "POST failed: %s", esp_err_to_name(err));

    //Closes the connection and frees up all the memory allocated to the HTTP client instance, call this in the end always
    esp_http_client_cleanup(client);
}



//This function pops from the CMP buffer at LT and sends data by calling send_packet function above
//It formats the packets with the time in #T format and applies base-64 encoding as well
void http_post_task(void *arg) {

    //create array of packets 
    static timestamped_packet_t batch_array[MAX_BATCH_PACKETS];

    static uint8_t post_payload[MAX_PAYLOAD_SIZE];
    static uint8_t base64_payload[MAX_PAYLOAD_SIZE * 2]; // Base64 can be up to 4/3 the size
    int packets_in_batch = 0;

    //LOG
    ESP_LOGI("HTTP_POST", "POST Task started. Waiting for CMP buffer data...");
    
    // Flag to wait for backlog processing
    //LOG
    bool backlog_wait_logged = false;
    //LOG
    while (ntp_synced && !backlog_complete) {
        if (!backlog_wait_logged) {
            ESP_LOGI("HTTP_POST", "Waiting for backlog processing to complete...");
            backlog_wait_logged = true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    //LOG
    if (backlog_wait_logged) {
        ESP_LOGI("HTTP_POST", "Backlog complete, starting transmission. CMP buffer has %lu packets", cmp_buffer_count());
    }

    while (1) {

        // Check CMP buffer level
        uint32_t cmp_count = cmp_buffer_count();
        
        // Pop packets from CMP buffer when LT is reached or timeout has occured since last time buffer was empty
        if (cmp_count >= CMP_BUFFER_LT || ((esp_timer_get_time() / 1000L) - cmp_fill_start >= send_data_timeout)) {

            //LOG
            ESP_LOGI("HTTP_POST", "DEBUG: MAX_BATCH_PACKETS = %d", MAX_BATCH_PACKETS);
            ESP_LOGI("HTTP_POST", "DEBUG: sizeof(batch_array) = %d", sizeof(batch_array));
            ESP_LOGI("HTTP_POST", "DEBUG: batch_array capacity = %d packets", sizeof(batch_array) / sizeof(timestamped_packet_t));
            packets_in_batch = 0;
            
            // Pop packets from CMP buffer
            while (packets_in_batch < MAX_BATCH_PACKETS && cmp_buffer_pop(&batch_array[packets_in_batch])) {
                packets_in_batch++;
            }
            
            if (packets_in_batch > 0) {
                ESP_LOGI("HTTP_POST", "Popped %d packets from CMP buffer (cmp_count=%lu)", packets_in_batch, cmp_buffer_count());
                
                size_t payload_len = 0;
                bool all_timestamped = true;
                
                // Check if all packets are timestamped
                for (int i = 0; i < packets_in_batch; i++) {
                    if (batch_array[i].sys_time == -1) {
                        all_timestamped = false;
                        break;
                    }
                }
                
                if (!all_timestamped) {
                    // BEFORE NTP SYNC: Send untimestamped packets
                    ESP_LOGI("HTTP_POST", "Sending UNTIMESTAMPED batch (NTP not synced yet)");
                    
                    for (int i = 0; i < packets_in_batch; i++) {
                        if (payload_len + batch_array[i].data_len <= MAX_PAYLOAD_SIZE) {
                            memcpy(post_payload + payload_len, batch_array[i].data, batch_array[i].data_len);
                            payload_len += batch_array[i].data_len;
                        }
                    }
                } 
                
                else {
                    // AFTER NTP SYNC: Send timestamped packets
                    ESP_LOGI("HTTP_POST", "Sending TIMESTAMPED batch - %d packets", packets_in_batch);
                    
                    // Log first packet's epoch time for verification
                    if (packets_in_batch > 0) ESP_LOGI("HTTP_POST", "First packet epoch: %lld (0x%08lX)", batch_array[0].sys_time, (unsigned long)batch_array[0].sys_time);
                    
                    for (int i = 0; i < packets_in_batch; i++) {
                        timestamped_packet_t *pkt = &batch_array[i];
                        
                        // Create #T timestamp header
                        char timestamp_header[16];
                        int header_len = snprintf(timestamp_header, sizeof(timestamp_header), "#T%08lX", (unsigned long)pkt->sys_time);
                        
                        //LOG : first timestamp header
                        if (i == 0) ESP_LOGI("HTTP_POST", "Timestamp header: '%s' (len=%d)", timestamp_header, header_len);
                        
                        // Check if we have space for header + data
                        if (payload_len + header_len + pkt->data_len <= MAX_PAYLOAD_SIZE) {
                            // Copy timestamp header
                            memcpy(post_payload + payload_len, timestamp_header, header_len);
                            payload_len += header_len;
                            
                            // Copy packet data
                            memcpy(post_payload + payload_len, pkt->data, pkt->data_len);
                            payload_len += pkt->data_len;
                        }
                    }
                }
                
                if (payload_len > 0) {
                    // Log payload in hex for debugging
                    //ESP_LOG_BUFFER_HEX("PAYLOAD", post_payload, payload_len);
                    
                    // Convert to Base64 before sending
                    size_t base64_len = 0;
                    int ret = mbedtls_base64_encode(base64_payload, sizeof(base64_payload), &base64_len, post_payload, payload_len);
                    if (ret == 0) {
                        send_packet_via_http(base64_payload, base64_len);

                        //LOG
                        ESP_LOGI("BASE64", "Encoded %d bytes to %d Base64 bytes", payload_len, base64_len);
                    } 
                    
                    //LOG
                    else ESP_LOGE("BASE64", "Encoding failed, error: %d", ret);
                }
            }
            
        } 
        
        // CMP buffer below LT, wait for more packets
        else vTaskDelay(pdMS_TO_TICKS(200));
    }
}
