#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "timestamp.h"
#include "uart.h"
#include "ring_buffer.h"
#include "compression.h"

// Max Wi-Fi APs to store
#define WIFI_MAX_AP 6

// NVS keys for storing last connected AP
#define LAST_SSID_KEY "last_ap"
#define LAST_PASS_KEY "last_pass"
#define WIFI_NAMESPACE "wifi_data"

// Timeout for WiFi connection attempt
#define WIFI_CONNECT_TIMEOUT_MS 8000

// Store AP credentials (since MAX_AP is 6, enter left out fields as NULL)
const char *ssid[WIFI_MAX_AP] = {"Bisquare", "iPhone", "testwifi", NULL, NULL, NULL};
const char *pass[WIFI_MAX_AP] = {"Ecco@41461oo", "YaThArTh", "11223344", NULL, NULL, NULL};

// NTP sync tracking variables, to store the time when system time was just updated
int64_t first_synced_epoch = -1;
int64_t first_synced_esp_ms = -1;

//flags to track sntp sync and data transfer
volatile bool ntp_synced = false;
volatile bool backlog_complete = false;

// RT Buffer (declared in main.c, defined here for external linkage)
QueueHandle_t rt_buffer;


// Store reference epoch time (1/1/2025 UTC) to check if sys_time was already updated
#define REF_EPOCH_TIME 1735689600
int64_t first_packet_systime;

//timeout to flush cmp buffer in msec
uint32_t send_data_timeout = 250000;
int64_t cmp_fill_start;

//LOGGING
static void log_buffer_status(const char* location) {
    uint32_t rt_count = uxQueueMessagesWaiting(rt_buffer);
    uint32_t cmp_count = cmp_buffer_count();
    
    float rt_util = (float)rt_count * 100.0f / RT_BUFFER_SIZE;
    float cmp_util = (float)cmp_count * 100.0f / CMP_BUFFER_CAPACITY;
    
    ESP_LOGI("BUFFER_STATUS", "[%s] RT: %lu/%d (%.1f%%) | CMP: %lu/%d (%.1f%%) | LT: %d", location, rt_count, RT_BUFFER_SIZE, rt_util,
             cmp_count, CMP_BUFFER_CAPACITY, cmp_util, CMP_BUFFER_LT);
    
    // Warning if buffers getting full
    if (rt_util > 80.0f) ESP_LOGW("BUFFER_STATUS", "RT Buffer >80%% full! Risk of packet loss!");

    if (cmp_util > 80.0f) ESP_LOGW("BUFFER_STATUS", "CMP Buffer >80%% full! HTTP POST may be slow!");
}



/*
Saving last AP details to NVS: this function attempts to save the WiFi network last connected with to Non-Voltile Storage
assigns values to the key using nvs APIs when a connection is made
*/
esp_err_t wifi_save_last_ap(const char* ssid, const char* pass) {
    //create nvs handle instance
    nvs_handle_t nvs;

    //open nvs for the namespace WIFI_NAMESPACE, in 'READWRITE' mode, and return the handle in the argument 'nvs'
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READWRITE, &nvs);

    //(using standard return type) if value of err is not ESP_OK, throw error
    if (err != ESP_OK) return err;
    
    //assign corresponding values to the nvs keys in the 'nvs' handle key-value pair dictionary
    nvs_set_str(nvs, LAST_SSID_KEY, ssid);
    nvs_set_str(nvs, LAST_PASS_KEY, pass);

    //write the changes to nvs and close
    nvs_commit(nvs);
    nvs_close(nvs);
    return ESP_OK;
}

/*
This function retrieves the details of the last connected ap from the NVS by performing NVS Read (nvs_get_str)
and stores them in ssid and pass fields
*/
bool wifi_load_last_ap(char* ssid, size_t ssid_len, char* pass, size_t pass_len) {
    nvs_handle_t nvs;

    //open nvs handle
    esp_err_t err = nvs_open(WIFI_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;
    
    //retrieve the SSID value using nvs_get_str() by giving LAST_SSID_KEY as the key value and store it in variable 'ssid'
    size_t ssid_size = ssid_len;
    err = nvs_get_str(nvs, LAST_SSID_KEY, ssid, &ssid_size);
    if (err != ESP_OK) {
        nvs_close(nvs);
        return false;
    }
    
    //similar to ssid, get the corresponding pass, store it and then close nvs
    size_t pass_size = pass_len;
    err = nvs_get_str(nvs, LAST_PASS_KEY, pass, &pass_size);
    nvs_close(nvs);
    return (err == ESP_OK);
}

/*
This function sorts the available Access Points based on RSSI value to determine the order of networks to connect to
Function parameters: Array to store sorted AP list, Maximum number of APs to scan
Return Number of APs found
*/
int ap_sort(wifi_ap_record_t* ap_list, int max_ap) {

    //provide the configuration parameters for wifi ssid scan
    wifi_scan_config_t scan_conf = {.ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = true};
    
    //scan all the available APs based on scan_conf config and store in WiFi driver, block the caller till scan is complete
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_conf, true));
    uint16_t ap_num = max_ap;

    //free the memory and Retrieve the list of APs found during the last scan. 
    //The esp_wifi_scan_get_ap_records() function returns the AP list in descending order based on RSSI.
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_list));
    
    return ap_num;
}

//This function attempts to connect to a given Wi-Fi AP given the credentials, return error handle
esp_err_t wifi_conn(const char* ssid, const char* pass) {

    //initialize wifi handle, store/set config data for the device's STA or AP
    wifi_config_t wifi_cfg = {0};

    //copy ssid and password into sta.ssid and sta.password fields
    //snprintf() function is used to print a specified string till a specified length in the specified format
    snprintf((char*)wifi_cfg.sta.ssid, sizeof(wifi_cfg.sta.ssid), "%s", ssid);
    snprintf((char*)wifi_cfg.sta.password, sizeof(wifi_cfg.sta.password), "%s", pass);
    
    //set wifi as station mode(STA), set STA config/interface, start wifi and connect to AP. LOG time of this event
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_connect());
    //ESP_LOGI("WIFI", "WiFi connect() called at %lld us", esp_timer_get_time());
    
    int elapsed = 0;
    while (elapsed < WIFI_CONNECT_TIMEOUT_MS) {

        //struct wifi_ap_record_t: description of wifi AP
        wifi_ap_record_t info;

        //Get information of AP to which the device is associated with, hold AP information in 'info'
        if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
            ESP_LOGI("WIFI", "Connected to SSID: %s / RSSI: %d at %lld us", ssid, info.rssi, esp_timer_get_time());
            return ESP_OK;
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
        elapsed += 500;
    }
    
    ESP_LOGW("WIFI", "Connection to %s failed", ssid);
    esp_wifi_disconnect();
    return ESP_FAIL;
}

/*
This function initializes the WiFi system and attempts connection. It tries last connected AP from NVS first, then scans and tries
available APs in order of RSSI strength
*/
static void wifi_init(void) {

    //init default nvs partition
    nvs_flash_init();

    //standard esp-netif initialization, use esp-netif network interfacing abstraction layer for managing wifi network interface
    //with tcp-ip stack (https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32c3/api-reference/network/esp_netif.html)
    esp_netif_init();
    esp_event_loop_create_default();

    //wifi default initialization using esp-netif
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // ATTEMPING CREDENTIALS STORED IN NVS
    char last_ssid[33] = {0}, last_pass[65] = {0};

    //use nvs load function above to get ssid and pass from nvs
    bool nvs_ok = wifi_load_last_ap(last_ssid, sizeof(last_ssid), last_pass, sizeof(last_pass));

    //LOG
    ESP_LOGI("NVS_DEBUG", "Loaded NVS values: nvs_ok=%d, last_ssid='%s'", nvs_ok, last_ssid);
    
    //if successful retrieve from nvs 
    if (nvs_ok && strlen(last_ssid) > 0) {

        //ESP_LOGI("WIFI", "Trying last connected SSID from NVS: %s", last_ssid);

        //attempt connection with nvs loaded credentials
        //if successful, save these AP details into nvs now and return from the function
        if (wifi_conn(last_ssid, last_pass) == ESP_OK) {
            ESP_LOGI("WIFI", "Connected with NVS credentials: %s", last_ssid);
            wifi_save_last_ap(last_ssid, last_pass);
            return;
        }

        ESP_LOGW("WIFI", "Could not connect to NVS stored credentials");
    } 
    
    //if no successful retrieve from nvs
    else ESP_LOGI("WIFI", "No valid NVS credentials found");
    


    // ATTEMPTING NORMAL WIFI CONNECT IN RSSI ORDER

    //retrieve the ap_list and sort
    wifi_ap_record_t ap_list[WIFI_MAX_AP] = {0};
    int ap_count = ap_sort(ap_list, WIFI_MAX_AP);
    
    //LOG available APs in RSSI order
    /*
    ESP_LOGI("WIFI_SCAN", "Available APs (by RSSI):");
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGI("WIFI_SCAN", "SSID: %s RSSI: %d", ap_list[i].ssid, ap_list[i].rssi);
    }*/
    
    //wifi connected flag, reset initially
    bool connected = false;
    for (int p = 0; p < WIFI_MAX_AP; p++) {

        //if no credentials at location p in the global ssid array, break
        if (ssid[p] == NULL) break;
        
        for (int s = 0; s < ap_count; s++) {

            //string compare the aplist and global ssid array and attempt connect
            if (strcmp((char*)ap_list[s].ssid, ssid[p]) == 0) {
                ESP_LOGI("CRED_ATTEMPT", "Trying SSID='%s'", ssid[p]);
                
                //if wifi_conn returns OK, save these credentials in the nvs now
                if (wifi_conn(ssid[p], pass[p]) == ESP_OK) {
                    ESP_LOGI("WIFI", "Connected to preferred SSID: %s", ssid[p]);
                    wifi_save_last_ap(ssid[p], pass[p]);

                    //set connected flag to indicate successful connection
                    connected = true;
                    break;
                }
            }
        }

        // connected is set here, break this loop and move forward
        if (connected) break;
    }
    
    //if connected is reset here, it means the break at line 312 was executed
    if (!connected) ESP_LOGW("WIFI", "Failed to connect to any network, global ssid array is empty...");
}


/*
This callback is invoked by the SNTP library when it receives a response from the NTP server. It captures the epoch time
and ESP timer time at sync moment for backward timestamping calculations.
parameter: tv pointer to timeval structure containing synchronized time
*/
void sntp_event_callback(struct timeval *tv) {
    //get value of esp timer when sync occured in msec
    int64_t now = esp_timer_get_time() / 1000L;
    
    // Capture first sync epoch and ESP timer values in msec
    first_synced_epoch = (int64_t)tv -> tv_sec * 1000L + (int64_t)tv -> tv_usec / 1000L;
    first_synced_esp_ms = now;
    
    // Set the NTP synced flag
    ntp_synced = true;
    
    // LOG: Monitor the print status
    sntp_sync_status_t status = sntp_get_sync_status();
    const char* status_str;
    switch(status) {
        case SNTP_SYNC_STATUS_RESET: status_str = "RESET"; break;
        case SNTP_SYNC_STATUS_COMPLETED: status_str = "COMPLETED"; break;
        case SNTP_SYNC_STATUS_IN_PROGRESS: status_str = "IN_PROGRESS"; break;
        default: status_str = "UNKNOWN"; break;
    }

    ESP_LOGI("SNTP_STATUS", "Sync status: %s", status_str);
    ESP_LOGI("SNTP", "NTP SYNCED! first_synced_epoch=%lld first_synced_esp_ms=%lld", first_synced_epoch, first_synced_esp_ms);
}

/*
This function configures SNTP servers, initializes SNTP and starts service
*/
void sntp_start(void *param) {
    
    //ESP_LOGI("SNTP", "Requesting SNTP sync at %lld us", esp_timer_get_time());
    
    // Configure SNTP
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    
    // Set SNTP servers
    esp_sntp_setservername(0, "pool.ntp.org");    // Pool of NTP servers
    esp_sntp_setservername(1, "in.pool.ntp.org");   //Pool of NTP servers, IN
    esp_sntp_setservername(2, "time.google.com");   // Google NTP
    esp_sntp_setservername(3, "time.aws.com");    // AWS NTP
    esp_sntp_setservername(4, "time.cloudflare.com");   //CloudFlare NTP
    
    //To view the sntp sync interval (how often sntp sync is requested, currently at 1 hour)
    //ESP_LOGI("SNTP", "Current SNTP sync interval: %lu ms", sntp_get_sync_interval());
    
    // Set callback for sync notification
    esp_sntp_set_time_sync_notification_cb(sntp_event_callback);
    
    // Initialize SNTP
    //ESP_LOGI("SNTP", "Initializing SNTP...");
    esp_sntp_init();
    //ESP_LOGI("SNTP", "SNTP init called");
    
    vTaskDelete(NULL);
}




void timestamp_task(void *param) {
    //ESP_LOGI("TIMESTAMP_TASK", "Started with highest priority");
    
    //create instance of packet struct 
    timestamped_packet_t packet;
    bool backlog_processed = false;

    ESP_LOGI("TIMESTAMP_TASK", "Started - monitoring buffers...");
    log_buffer_status("STARTUP");

    static uint32_t packet_counter = 0;

    //time when cmp buffer starts to fill
    //if (cmp_buffer_count < 2) cmp_fill_start = esp_timer_get_time() / 1000L;
    
    while (1) {

        //mark the time when cmp buffer starts to fill (to check buffer flush timeout)
        if (cmp_buffer.count < 2) cmp_fill_start = esp_timer_get_time() / 1000L;

        //Get the NTP system time using gettimeofday(), and check if system time is already updated using epoch time
        struct timeval tv_check;
        gettimeofday(&tv_check, NULL);
        bool time_is_valid = (tv_check.tv_sec >= REF_EPOCH_TIME);
        
        //if either ntp sync flag is set or if system time was already updated, but there are untimestamped packets
        if ((ntp_synced || time_is_valid) && !backlog_processed) {

            //LOG: show the first synced times
            ESP_LOGI("TIMESTAMP_TASK", "NTP Sync occured, Processing backlog...");
            ESP_LOGI("TIMESTAMP_TASK", "first_synced_epoch (msec) = %lld (0x%08lX)", first_synced_epoch, (unsigned long)first_synced_epoch);
            ESP_LOGI("TIMESTAMP_TASK", "first_synced_esp_ms = %lld", first_synced_esp_ms);
            
            // If NTP didn't sync but system time is valid, capture reference times now 
            if (!ntp_synced && time_is_valid) {
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                first_synced_epoch = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
                first_synced_esp_ms = esp_timer_get_time() / 1000L;
                ESP_LOGI("TIMESTAMP_TASK", "System time already set, Using as reference.");
                ESP_LOGI("TIMESTAMP_TASK", "first_synced_epoch = %lld (0x%08lX)", first_synced_epoch, (unsigned long)first_synced_epoch);
                ESP_LOGI("TIMESTAMP_TASK", "first_synced_esp_ms = %lld", first_synced_esp_ms);
            }
            
            //LOG: number of packets filled in RT buffer
            int backlog_count = 0;
            int rt_count_before = uxQueueMessagesWaiting(rt_buffer);
            ESP_LOGI("TIMESTAMP_TASK", "RT buffer has %d packets", rt_count_before);

            //calculate timer_delta, delta between esp timer and system timer. Use first packet esp timer time stored globally
            //and use first packet system time as well to get the delta
            static bool first_packet_systime_got = false;
            if (!first_packet_systime_got) {
                timestamped_packet_t first_packet;
                xQueuePeek(rt_buffer, &first_packet, portMAX_DELAY);
                first_packet_systime = first_packet.sys_time;
                first_packet_systime_got = true;
            }

            int64_t timer_delta = esp_time_mstest - first_packet_systime;

            //All packets so far in RT buffer were captured before sync
            //introduce a loop to shift packet-by-packet to struct and do backward timestamp
            while (xQueueReceive(rt_buffer, &packet, 0) == pdTRUE) {

                //the untimestamped packet system time before sync
                int64_t presync_systime = packet.sys_time;

                //the delta/drift between system time and esp timer time (esp timer is always ahead of system timer)
                ///int64_t timer_delta = esp_time_mstest - first_packet_systime;

                //backward timestamping formula, all calculations in msec, and finally converted to seconds
                //first synced epoch time in ms - first synced esp timer time gives epoch time of esp timer init
                //add to this the pre-sync system time of first packet, and then add drift between both timers
                packet.sys_time = (first_synced_epoch - first_synced_esp_ms + presync_systime + timer_delta) / 1000L;
                
                uint8_t compressed_pkt[MAX_BINARY_PACKET_SIZE];
                size_t compressed_len = 0;
                
                compress_packet(packet.data, packet.data_len, compressed_pkt, &compressed_len);
                
                // Only push if compression produced output (compressed_len > 0)
                if (compressed_len > 0) {
                    memcpy(packet.data, compressed_pkt, compressed_len);
                    packet.data_len = compressed_len;
                    
                    if (!cmp_buffer_push(&packet)) ESP_LOGW("TIMESTAMP_TASK", "CMP buffer full during backlog! Packet dropped.");
                    backlog_count++;
                }
                
                if (backlog_count % 20 == 0 && backlog_count > 0) log_buffer_status("BACKLOG");
            }
            
            //While loop for backward timestamping completed
            ESP_LOGI("TIMESTAMP_TASK", "Backlog complete: %d packets timestamped", backlog_count);
            ESP_LOGI("TIMESTAMP_TASK", "CMP buffer now has %lu packets", cmp_buffer_count());
            ESP_LOGI("TIMESTAMP_TASK", "RT buffer utilization: %d/%d packets", uxQueueMessagesWaiting(rt_buffer), RT_BUFFER_SIZE);

            //set flags for backward timestamping completion
            backlog_processed = true;
            backlog_complete = true;
            
            // Continue to main while loop for further processing
            continue;
        }
        
        //if time not valid yet, wait and fill RT buffer itself
        else if (!time_is_valid) vTaskDelay(pdMS_TO_TICKS(100));
        
        //if time is valid and no untimestamped backlog in buffer
        else {
            if (xQueueReceive(rt_buffer, &packet, pdMS_TO_TICKS(10)) == pdTRUE) {
                packet_counter++;
                
                // Timestamp packet
                struct timeval tv_now;
                gettimeofday(&tv_now, NULL);
                packet.sys_time = tv_now.tv_sec;
                
                // *** CHECK FORCE TIMEOUT ***
                check_force_timeout(packet.sys_time);
                
                // *** ADD COMPRESSION HERE ***
                uint8_t compressed_pkt[MAX_BINARY_PACKET_SIZE];
                size_t compressed_len = 0;
                
                compress_packet(packet.data, packet.data_len, compressed_pkt, &compressed_len);
                
                // Only push if compression produced output
                if (compressed_len > 0) {
                    ESP_LOGI("TIMESTAMP_TASK", "Pkt %lu: Compressed %d→%d, pushing to CMP (count will be %lu)",
                            packet_counter, packet.data_len, compressed_len, cmp_buffer_count() + 1);

                    memcpy(packet.data, compressed_pkt, compressed_len);
                    packet.data_len = compressed_len;
                    
                    if (!cmp_buffer_push(&packet)) ESP_LOGW("TIMESTAMP_TASK", "CMP buffer full! Packet dropped.");
                }

                else {
                    ESP_LOGI("TIMESTAMP_TASK", "Pkt %lu: SKIPPED (len=0), CMP count stays at %lu",
                    packet_counter, cmp_buffer_count());
                }
            } 
            else {
                // No packet received within 10ms, sleep briefly before trying again
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}


/*
This function initializes the CMP buffer, WiFi connection, and SNTP service.
*/
void time_init(void) {
    ESP_LOGI("TIMER", "Timestamp system initialized at %lld us", esp_timer_get_time());
    
    cmp_buffer_init();
    compression_init();
    wifi_init();
    
    // Start SNTP task
    xTaskCreate(sntp_start, "sntp_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
}
