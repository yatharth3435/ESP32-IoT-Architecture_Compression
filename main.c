#include "uart.h"
#include "timestamp.h"
#include "esp_log.h"
#include "post_data.h"

//this size is declared in terms of "number of items" (here packets)
#define RT_BUFFER_SIZE 50
extern QueueHandle_t rt_buffer;

//FreeRTOS task stack size
#define CONFIG_TASK_STACK_SIZE 4096


void app_main(void)
{
    //log the size of the struct of one packet and the free heap available at startup
    ESP_LOGI("QUEUE", "timestamped_packet_t size: %u", sizeof(timestamped_packet_t));
    ESP_LOGI("QUEUE", "Free heap at startup: %lu", esp_get_free_heap_size());

    rt_buffer = xQueueCreate(RT_BUFFER_SIZE, sizeof(timestamped_packet_t));

    //if rt_buffer could not be allocated, log error and return
    if (rt_buffer == NULL) {
        ESP_LOGE("MAIN", "Fatal Error: Failed to create rt_buffer. Not enough heap memory.");
        return;
    } 
    
    else {
        ESP_LOGI("MAIN", "rt_buffer of size %d created successfully.", RT_BUFFER_SIZE);
    }

    uart_init();  //initialize uart
    time_init(); 


    // rx_task has high priority (receives UART data)
    xTaskCreate(rx_task, "uart_rx", CONFIG_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);


    xTaskCreate(timestamp_task, "timestamp_task", CONFIG_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
    
    // tx_uart has lower priority (transmits test data)
    xTaskCreate(tx_uart, "uart_tx", CONFIG_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 3, NULL); 
    
    // http_post_task has lower priority (sends data to server from CMP buffer)
    xTaskCreate(http_post_task, "http_post_task", CONFIG_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 4, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Sleep for 10 seconds
    }
}