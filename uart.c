
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sntp.h"
#include "sys/time.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "timestamp.h"

/*
This file does the following implementations:-
- UART RX/TX transfer tasks
- ASCII to binary conversion(binary compression)
- RT Buffer with initial timestamping
*/

int64_t esp_time_mstest;

//the size of uart rx/tx ring buffer, where the ISR reads/writes data from the UART FIFO. 
static const int UART_RX_BUF_SIZE = 1024;
static const int UART_TX_BUF_SIZE = 512;
QueueHandle_t uart_queue;

//pins to receive and transmit UART data, put #definitions in a .h file separately
#define TXD_PIN (21) 
#define RXD_PIN (20)

//stack size for Tasks
#define CONFIG_TASK_STACK_SIZE 4096

//init function to initialize all protocols
void uart_init(void) 
{
    //standard procedure to init uart
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_RX_BUF_SIZE * 2, UART_TX_BUF_SIZE, 10, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/*
This function takes in a group of two hex encoded ascii characters(2 bytes) and typecasts them to binary(1 byte) 
*/
int ascii_hex2bin(const uint8_t *ascii_hex, uint8_t *bin_out)
{
    unsigned int value;
    // Expect ascii_hex to be a pointer to two ASCII hex characters (e.g., "FF")
    if (sscanf((const char *)ascii_hex, "%2x", &value) == 1) {
        *bin_out = (uint8_t)value;
        return 0; // if 2 ascii characters were found and parsing done successfully
    }
    
    // error (parsing failed) if some invalid hex character found
    return -1;
}

/*
Binary Conversion algorithm: Converts the oncoming hex-coded ascii_buf serial data into pure binary data.
The purpose of the function is to half-compress the data (size of one hex-coded ascii_buf = 1 byte, size in binary = 1 nibble)
#,s,m,c characters are not converted, others are converted since they are all numbers. sscanf() function is used for typecasting.
input parameters of the function are the ascii_buf string and an array to store binary data.

int bin_conv(const uint8_t *ascii_buf[input ascii string], uint8_t *bin_buf[output buffer storing binary data])
*/
int bin_conv(const uint8_t *ascii_buf, uint8_t *bin_buf) { 

    int i = 0;
    int bin_len = 0;

    bin_buf[bin_len++] = ascii_buf[i++];
    bin_buf[bin_len++] = ascii_buf[i++];

    while (ascii_buf[i] != '\r' && ascii_buf[i] != '\n') {
        //ensure that only actual typecaste-able characters (i.e. numbers) are converted
        if ((ascii_buf[i] >= '0' && ascii_buf[i] <= '9') || (ascii_buf[i] >= 'A' && ascii_buf[i] <= 'F')) {
            if ((ascii_buf[i+1] >= '0' && ascii_buf[i+1] <= '9') || (ascii_buf[i+1] >= 'A' && ascii_buf[i+1] <= 'F')) {
                if (ascii_hex2bin(&ascii_buf[i], &bin_buf[bin_len]) == 0) {
                    bin_len++;
                }
                i += 2;
                continue;
            }
        }
    bin_buf[bin_len++] = ascii_buf[i++];
    }
    return bin_len;
}

//This function carries out tx functions(IoT to IDU), currently string data
int send_data_to_UART(const char* logName, const char* data)
{
    //store the length of the data in a variable len
    const int len = strlen(data);

    //call uart write function to write to uart tx ring buffer
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);

    //ESP_LOGI(logName, "Wrote %d bytes", txBytes);

    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, 100));
    return txBytes;
}

//this function does the actual uart transmission by calling send_data_to_uart
void tx_uart(void *arg)
{
    //assigning a tag to the task, to show the process is under this task while logging
    static const char *TX_TASK_TAG = "TX_UART";

    //set the log level, it is an info type log
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    //call send_data_to_uart() and transmit at interval 2 seconds
    while (1) {
        send_data_to_UART(TX_TASK_TAG, "UART TX data (IoT to IDU)");  //25 bytes of data
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void rx_task(void *arg)
{
    //create tag name for logging, under uart_rx task and set the log level
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    //variable to store raw data received from uart (also use for testing data receive)
    uint8_t data[UART_RX_BUF_SIZE]; 

    //to store binary compressed data
    uint8_t binary[UART_RX_BUF_SIZE / 2];  
    
    while (1) {

        //carry out uart read, on port UART_NUM_1, till UART_RX_BUF_SIZE and wait for 10msec for incoming data before freeing
        //RTOS tick rate: 100Hz i.e. 100 ticks per second i.e. 1 tick = 10msec
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, UART_RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);

        if (rxBytes > 0) {
            //null termination for string end
            data[rxBytes] = 0;

            //Capture timestamp once (in msec) so that any bunched #m/#c packets get same time
            ///int64_t esp_time_ms = esp_timer_get_time() / 1000L;
            
            //timeval struct to obtain time
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            
            //Calculate system time in ms (if sntp sync hasn't occured, system time stores offset since system start)
            //formula: append 000 after seconds time, and add microsecond/1000 (ms) to this to get msec time 
            int64_t sys_time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;


            //Capture esp timer time of only the first packet for reference (to get delta between system time and esp timer time)
            static bool first_packet_captured = false;
            if (!first_packet_captured) {
                esp_time_mstest = esp_timer_get_time() / 1000L;
                first_packet_captured = true;
                
            }

            //LOG how many bytes of data received in total
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes total", rxBytes);
            //ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            //set this flag to activate binary compression, reset it to bypass(FOR TESTING)
            int activate_bin_conv = 1;

            //Process all packets in the received buffer from here now (after timestamping)
            //offset is the universal location of rx data, in the data we recognize CR LF and get packet_len
            int offset = 0;
            while (offset < rxBytes) {
                
                //a variable storing packet length to extract one packet per call and identify CR LF
                int packet_len = 0;      

                //keep incrementing 'i' till CR LF detected, so it automatically stores packet length if we break at termination
                for (int i = offset; i < rxBytes; i++) {
                    if (data[i] == '\n' || data[i] == '\r' || data[i] == '\0') {
                        packet_len = i - offset; 
                        
                        // terminate packet as soon as CR LF detected
                        break;
                    }
                }

                //if no delimiter found, we've reached end of complete packets
                if (packet_len == 0) {
                    break;
                }

                //in a variable 'save', store what is after the end of this packet, and temporarily null terminate this
                //packet for efficient string handling (for LOG and for processing in bin_conv())
                uint8_t save = data[offset + packet_len];
                data[offset + packet_len] = 0;

                //log the actual packet received (only run during debugging/testing)
                //ESP_LOGI(RX_TASK_TAG, "Processing packet at offset %d, length %d: '%s'", offset, packet_len, &data[offset]);

                //declare a variable based on defined struct for packet, to store data and initial time
                timestamped_packet_t pkt_out;

                if (activate_bin_conv) {
                    //carry out binary conversion
                    int bin_len = bin_conv(&data[offset], binary);

                    //assign the data_len field appropriately and transfer the data to the packet data field from 'binary'
                    pkt_out.data_len = bin_len;
                    memcpy(pkt_out.data, binary, bin_len);

                    //display bin_len
                    ESP_LOGI(RX_TASK_TAG, "The data is now converted to %d binary bytes and stored as packet", bin_len);

                    //log the binary packet (only for test/debug)
                    //ESP_LOG_BUFFER_HEXDUMP("BIN_PACKET", binary, bin_len, ESP_LOG_INFO);
                }

                //if activate_bin_conv = 0, we dont want to do binary conversion, packet goes in hex_encoded_ascii as it is
                else {
                    //assign length directly from packet_len in that iteration
                    //also directly transfer data variable to the packet data field
                    pkt_out.data_len = packet_len;
                    memcpy(pkt_out.data, &data[offset], packet_len);
                }

                //Use the SAME timestamp captured at the beginning for the packet/s arrived
                ///pkt_out.esp_time_ms = esp_time_ms;
                pkt_out.sys_time = sys_time_ms;

                // LOG the timestamps
                ESP_LOGI(RX_TASK_TAG, "Packet timestamped: sys_time: %lld ms, first packet esp time: %lld", pkt_out.sys_time, esp_time_mstest);

                //Send the data to rt buffer 
                //if (rt_buffer != NULL) xQueueSend(rt_buffer, &pkt_out, 10 / portTICK_PERIOD_MS);
                if (rt_buffer != NULL) {

                    //send data from pkt_out to rt_buffer, packetwise
                    //TickstoWait field: The maximum amount of time the task should block waiting for space to become 
                    //available on the queue, should it already be full
                    BaseType_t result = xQueueSend(rt_buffer, &pkt_out, pdMS_TO_TICKS(100));

                    if (result != pdTRUE) {
                        ESP_LOGE(RX_TASK_TAG, "FAILED to send packet to RT buffer! Queue full or timeout!");
                    } 
                    
                    else {
                        ESP_LOGI(RX_TASK_TAG, "Packet sent to RT buffer, currently has %d packets", uxQueueMessagesWaiting(rt_buffer));
                    }
                }

                //retrieve the element in the save variable and allot it to its original position for next processing
                data[offset + packet_len] = save;
                
                //Move to next packet in the buffer (Skip CR LF)
                offset += packet_len + 2;
            }
        }
    }
    free(data);
    free(binary);
}