#include "compression.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "COMPRESSION";

// Compression control flags
bool compression_enable = true;
bool force_pkt_push_flag = false;
bool send_full_status_pkt_flag = true;   // Send full packet initially
bool send_full_mode_pkt_flag = true;     // Not used (mode never compressed)
bool por_update_status_pilot_flag = true;  // Update pilot on first packet
bool por_update_mode_pilot_flag = true;

//LOG VARIABLES
static uint32_t total_packets_processed = 0;
static uint32_t full_packets_sent = 0;
static uint32_t compressed_packets_sent = 0;
static uint32_t packets_skipped = 0;
static uint32_t total_bytes_in = 0;
static uint32_t total_bytes_out = 0;

// Force packet timeout (120 seconds = 2 minutes)
uint32_t force_pkt_timeout = 120;

// Timestamp tracking
int64_t last_forced_pkt_time = 0;

// Packet structure
// Byte 0-1:  '#s'
// 2-3:  Payload size
// 4-5:  Sequence number (NOT a parameter, included in payload length)
// 6+:   26 parameters (each 2 bytes)

uint16_t thresholds[NUM_OF_TOTAL_PARAMS] = {
    // MODE PACKET PARAMS (indices 0-17) - threshold = 0 (no compression)
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    
    // Param 0: IDU Fan Speed (RPM)
    0, 
    
    // and so on (lossless comp.)
    0,  0,  0,  0xFFFF /*Compressor timer, neglected param*/,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    
    /*Neglected params, Debug registers*/
    0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

// Minimum valid values (prev. minor thresholds) - Set to 0 for no
uint16_t min_val[NUM_OF_TOTAL_PARAMS] = {0};

// Parameter types
uint8_t param_types[NUM_OF_TOTAL_PARAMS] = {
    // MODE PACKET: All NON_CRITICAL (but never compressed due to threshold=0)
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    
    // STATUS PACKET (26 parameters, indices 18-43):
    NON_CRITICAL_PARAM/*IDU Fan Speed*/, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    NEGLECTED_PARAM/*Compressor Timer*/, 
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,  NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, NON_CRITICAL_PARAM,
    NON_CRITICAL_PARAM, NON_CRITICAL_PARAM, CRITICAL_PARAM, CRITICAL_PARAM, NEGLECTED_PARAM, NEGLECTED_PARAM, NEGLECTED_PARAM,
    NEGLECTED_PARAM, NEGLECTED_PARAM
};

// Pilot baseline storage
uint16_t pilot_status_params[NUM_OF_STATUS_PARAMS] = {0};
uint16_t pilot_mode_params[NUM_OF_MODE_PARAMS] = {0};

// Current packet parameter storage
uint16_t current_status_params[NUM_OF_STATUS_PARAMS] = {0};
uint16_t current_mode_params[NUM_OF_MODE_PARAMS] = {0};

//Get absolute value of signed 16-bit integer (for parameters that can have negative value)
int16_t abs_s(int16_t number) {
    return (number >= 0) ? number : -number;
}

//extract 16 bit substring, this is done to extract parameter from packet and check compression
uint16_t extract_uint16(const uint8_t *data) {
    return ((uint16_t)data[0] << 8) | data[1];
}

//Write 16-bit value to binary buffer in big-endian format
void write_uint16(uint8_t *data, uint16_t value) {
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;
}

//Initialise compression system
void compression_init(void) {
    //ESP_LOGI(TAG, "Compression system initialized");
    ESP_LOGI(TAG, "Compression enabled: %d", compression_enable);
    ESP_LOGI(TAG, "Force packet timeout: %lu seconds", force_pkt_timeout);
    ESP_LOGI(TAG, "Status params: %d, Mode params: %d", NUM_OF_STATUS_PARAMS, NUM_OF_MODE_PARAMS);
    
    // Reset pilot update flags
    por_update_status_pilot_flag = true;
    por_update_mode_pilot_flag = true;
    send_full_status_pkt_flag = true;
    send_full_mode_pkt_flag = true;
    
    // Initialize last forced packet time to 0
    last_forced_pkt_time = 0;
}

//Check if force packet timeout has expired. If timeout expired, set flags to send full pilot packet and update baseline.
void check_force_timeout(int64_t current_time_sec) {
    // Skip the check if last_forced_pkt_time not set yet
    if (last_forced_pkt_time == 0) {
        last_forced_pkt_time = current_time_sec;
        return;
    }
    
    // Check if timeout expired
    if ((current_time_sec - last_forced_pkt_time) >= force_pkt_timeout) {
        ESP_LOGI(TAG, "Force timeout expired (%lld sec), forcing pilot refresh", current_time_sec - last_forced_pkt_time);
        
        // Set flags to send full status and mode packet and update pilot baseline
        send_full_status_pkt_flag = true;
        send_full_mode_pkt_flag = true;
        force_pkt_push_flag = true;
        
        // Reset timer
        last_forced_pkt_time = current_time_sec;
    }
}

/*
Process the packet parameters one by one and see if they are to be compressed
packet: Input binary packet
context: Compression context with pilot data and thresholds
payload_out: Output buffer for compressed payload
payload_len: Pointer to output payload length
*/
void process_packet_params(const uint8_t *packet, CompressionContext *context, uint8_t *payload_out, size_t *payload_len) {
    
    *payload_len = 0;

    //params start at 6th byte, after #, s, payload size(2), seq num(2)
    const uint8_t *params_start = packet + 6;
    
    //iterate through all params
    for (uint16_t i = 0; i < context->param_count; i++) {

        // Extract 2-byte parameter value using substring function
        uint16_t param_value = extract_uint16(&params_start[i * 2]);
        
        // Apply min_val, i.e. if the param value is <= the min_val (negligible), substitute the param value with min_val 
        if (param_value <= context->min_val[context->param_ptr + i]) param_value = context->min_val[context->param_ptr + i];
        
        // Check if parameter should be processed (not NEGLECTED param)
        if (context->param_types[context->param_ptr + i] != NEGLECTED_PARAM) {
            
            //FULL packet send (acc to compression context)
            if (context->is_full_pkt) {
                // Update current parameter value
                context->current_params[i] = param_value;
                
                // If pilot update flag is set, also update pilot baseline
                if (*context->pilot_update_flag) context->pilot_params[i] = param_value;
            } 

            //COMPRESSION
            else {

                // Calculate absolute delta from current baseline
                int16_t delta = context->current_params[i] - param_value;
                uint16_t absolute_delta = abs_s(delta);
                
                // Check if delta exceeds threshold (0 for lossless)
                if (absolute_delta > context->thresholds[context->param_ptr + i]) {
                    // Parameter changed - update current value
                    context->current_params[i] = param_value;
                    
                    //Check if we have space in output buffer (+3 because adding to #k occurs in 3 byte increments, 1 byte for
                    //index and 2 for parameter value itself)
                    if (*payload_len + 3 <= MAX_COMPRESSED_PAYLOAD_SIZE) {
                        // Write parameter index (1 byte) - compression.cpp line 438
                        // Use absolute index from arrays (param_ptr + i)
                        //payload_out[*payload_len] = context->param_ptr + i;
                        //(*payload_len)++;

                        //for status packet, offset 32
                        if (context->param_ptr == STATUS_PKT_PTR) payload_out[*payload_len] = 32 + i;
                        //for mode packet, start from 0
                        else payload_out[*payload_len] = 0 + i;

                        (*payload_len)++;
                        
                        // Write parameter value (2 bytes, big-endian) - compression.cpp line 443
                        write_uint16(&payload_out[*payload_len], param_value);
                        (*payload_len) += 2;
                    }
                }
                // else: delta within threshold, don't include in compressed packet
            }
        }
    }
    
    //clear pilot update flag after the entire loop completes
    if (*context->pilot_update_flag) *context->pilot_update_flag = false;
}


/*
This function takes in an input packet, uses process_params() function to compress params, and stores the packets at packet_out
Compressed packet format: '#' - 'k' - payload size(2B) - Seq num(2B) - [idx][val][val][idx][val][val]...
*/
void compress_packet(const uint8_t *packet_in, size_t packet_in_len, uint8_t *packet_out, size_t *packet_out_len) {
    
    // Check if compression is globally disabled
    if (!compression_enable) {
        // Return packet as-is
        memcpy(packet_out, packet_in, packet_in_len);
        *packet_out_len = packet_in_len;
        return;
    }
    
    // Validate minimum packet size (header + size + seq_num)
    if (packet_in_len < 6) {
        ESP_LOGW(TAG, "Packet too short: %d bytes", packet_in_len);
        memcpy(packet_out, packet_in, packet_in_len);
        *packet_out_len = packet_in_len;
        return;
    }
    
    // Extract packet type
    char pkt_type = packet_in[1];
    
    // Extract payload size and sequence number
    uint16_t payload_size = extract_uint16(&packet_in[2]);
    uint16_t seq_num = extract_uint16(&packet_in[4]);
    
    // CONFIG PACKETS: Always send as-is
    if (pkt_type == CONFIG_PKT_CODE) {
        memcpy(packet_out, packet_in, packet_in_len);
        *packet_out_len = packet_in_len;
        ESP_LOGD(TAG, "Config packet - sent as-is");
        return;
    }
    
    // MODE PACKETS: Compress but always with 0 threhsold
    if (pkt_type == MODE_PKT_CODE) {
        /*
        memcpy(packet_out, packet_in, packet_in_len);
        *packet_out_len = packet_in_len;
        ESP_LOGD(TAG, "Mode packet - sent as-is (no compression)");
        return;
        */

        CompressionContext context = {
            .pilot_params = pilot_mode_params,
            .current_params = current_mode_params,
            .thresholds = thresholds,
            .min_val = min_val,
            .param_types = param_types,
            .param_ptr = MODE_PKT_PTR,
            .param_count = NUM_OF_MODE_PARAMS,
            .pilot_update_flag = &por_update_mode_pilot_flag,
            .is_full_pkt = send_full_mode_pkt_flag || force_pkt_push_flag
        };

        uint8_t compressed_payload[MAX_COMPRESSED_PAYLOAD_SIZE];
        size_t compressed_len = 0;

        process_packet_params(packet_in, &context, compressed_payload, &compressed_len);

        // Check if we should send full packet or compressed packet
        if (context.is_full_pkt) {
            // SEND FULL PACKET (pilot baseline update)
            memcpy(packet_out, packet_in, packet_in_len);
            *packet_out_len = packet_in_len;
            
            // Clear flags
            send_full_mode_pkt_flag = false;
            force_pkt_push_flag = false;
            
            ESP_LOGI(TAG, "Mode packet - FULL (pilot baseline updated)");
        }

        else if (compressed_len == 0) {
            // NO PARAMETERS CHANGED - Don't send packet at all
            *packet_out_len = 0;
            ESP_LOGD(TAG, "Mode packet - no changes, skipping");
        }

        else {
            // SEND COMPRESSED PACKET
            
            // Build compressed packet header
            packet_out[0] = '#';
            packet_out[1] = COMPRESSED_PKT_CODE;  // 'k' (same as status)
            
            // Calculate compressed payload size (compressed data + sequence number)
            uint16_t actual_payload_size = compressed_len + 2;
            write_uint16(&packet_out[2], actual_payload_size);
            
            // Copy sequence number (ALWAYS included)
            write_uint16(&packet_out[4], seq_num);
            
            // Copy compressed payload
            memcpy(&packet_out[6], compressed_payload, compressed_len);
            
            // Set output length
            *packet_out_len = 6 + compressed_len;
            
            ESP_LOGI(TAG, "Mode packet - COMPRESSED: %d params, %d bytes (was %d bytes)", compressed_len / 3, *packet_out_len, packet_in_len);
        }

        return;
    }
    
    // STATUS PACKETS: Apply compression
    if (pkt_type == STATUS_PKT_CODE) {
        
        // Prepare compression context
        CompressionContext context = {
            .pilot_params = pilot_status_params,
            .current_params = current_status_params,
            .thresholds = thresholds,
            .min_val = min_val,
            .param_types = param_types,
            .param_ptr = STATUS_PKT_PTR,
            .param_count = NUM_OF_STATUS_PARAMS,
            .pilot_update_flag = &por_update_status_pilot_flag,
            .is_full_pkt = send_full_status_pkt_flag || force_pkt_push_flag
        };
        
        // Buffer for compressed payload
        uint8_t compressed_payload[MAX_COMPRESSED_PAYLOAD_SIZE];
        size_t compressed_len = 0;
        
        // Process parameters
        process_packet_params(packet_in, &context, compressed_payload, &compressed_len);
        
        // Check if we should send full packet or compressed packet
        if (context.is_full_pkt) {
            // SEND FULL PACKET (pilot baseline update)
            memcpy(packet_out, packet_in, packet_in_len);
            *packet_out_len = packet_in_len;
            
            // Clear flags
            send_full_status_pkt_flag = false;
            force_pkt_push_flag = false;
            
            ESP_LOGI(TAG, "Status packet - FULL (pilot baseline updated)");
        }
        else if (compressed_len == 0) {
            // NO PARAMETERS CHANGED - Don't send packet at all
            *packet_out_len = 0;
            ESP_LOGD(TAG, "Status packet - no changes, skipping");
        }
        else {
            // SEND COMPRESSED PACKET
            
            // Build compressed packet header
            packet_out[0] = '#';
            packet_out[1] = COMPRESSED_PKT_CODE;  // 'k'
            
            // Calculate compressed payload size (compressed data)
            uint16_t actual_payload_size = compressed_len + 2;  // what goes into bytes 2–3

            //after 'k', write as much as compressed_len
            write_uint16(&packet_out[2], actual_payload_size);
            
            // Copy sequence number (ALWAYS included)
            write_uint16(&packet_out[4], seq_num);
            
            // Copy compressed payload
            memcpy(&packet_out[6], compressed_payload, compressed_len);
            
            // Set output length
            *packet_out_len = 6 + compressed_len;
            
            ESP_LOGI(TAG, "Status packet - COMPRESSED: %d params, %d bytes (was %d bytes)", compressed_len / 3, *packet_out_len, packet_in_len);
        }
        
        return;
    }
    
    // Unknown packet type - send as-is
    ESP_LOGW(TAG, "Unknown packet type '%c', sending as-is", pkt_type);
    memcpy(packet_out, packet_in, packet_in_len);
    *packet_out_len = packet_in_len;

    total_packets_processed++;
    total_bytes_in += packet_in_len;
    
    if (*packet_out_len == 0) {
        packets_skipped++;
        ESP_LOGD("COMPRESSION", "Packet skipped (no changes)");
    } else {
        total_bytes_out += *packet_out_len;
        
        if (packet_out[1] == COMPRESSED_PKT_CODE) {
            compressed_packets_sent++;
            ESP_LOGI("COMPRESSION", "Packet compressed: %d → %d bytes (%.1f%% reduction)",
                     packet_in_len, *packet_out_len,
                     (1.0f - ((float)*packet_out_len / packet_in_len)) * 100.0f);
        } else {
            full_packets_sent++;
            ESP_LOGI("COMPRESSION", "Packet sent FULL: %d bytes (type=%c)",
                     *packet_out_len, packet_out[1]);
        }
    }
    
    // Log stats every 50 packets
    if (total_packets_processed % 50 == 0) log_compression_stats();
}


void log_compression_stats(void) {
    if (total_packets_processed == 0) {
        ESP_LOGI("COMP_STATS", "No packets processed yet");
        return;
    }
    
    float compression_ratio = 0;
    if (total_bytes_in > 0) {
        compression_ratio = (1.0f - ((float)total_bytes_out / (float)total_bytes_in)) * 100.0f;
    }
    
    ESP_LOGI("COMP_STATS", "=== Compression Statistics ===");
    ESP_LOGI("COMP_STATS", "Total packets: %lu", total_packets_processed);
    ESP_LOGI("COMP_STATS", "  - Full: %lu (%.1f%%)", full_packets_sent, 
             (float)full_packets_sent * 100.0f / total_packets_processed);
    ESP_LOGI("COMP_STATS", "  - Compressed: %lu (%.1f%%)", compressed_packets_sent,
             (float)compressed_packets_sent * 100.0f / total_packets_processed);
    ESP_LOGI("COMP_STATS", "  - Skipped: %lu (%.1f%%)", packets_skipped,
             (float)packets_skipped * 100.0f / total_packets_processed);
    ESP_LOGI("COMP_STATS", "Bytes: %lu → %lu (%.1f%% reduction)", 
             total_bytes_in, total_bytes_out, compression_ratio);
    ESP_LOGI("COMP_STATS", "Compression enabled: %s", compression_enable ? "YES" : "NO");
    ESP_LOGI("COMP_STATS", "===============================");
}