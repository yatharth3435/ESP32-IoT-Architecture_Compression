#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>



// Number of parameters in each packet type
#define NUM_OF_STATUS_PARAMS        26    // #s packet: 26 parameters (param 0-25)
#define NUM_OF_MODE_PARAMS          18    // #m packet: 18 parameters (param 0-17)
#define NUM_OF_CONFIG_PARAMS        7     // #c packet: 7 fields
#define NUM_OF_TOTAL_PARAMS         (NUM_OF_STATUS_PARAMS + NUM_OF_MODE_PARAMS)

// Packet type codes (byte 1 in packet)
#define MODE_PKT_CODE               'm'
#define CONFIG_PKT_CODE             'c'
#define STATUS_PKT_CODE             's'
#define COMPRESSED_PKT_CODE         'k'

// Parameter array indices
#define MODE_PKT_PTR                0                           // Mode params start at index 0
#define STATUS_PKT_PTR              NUM_OF_MODE_PARAMS          // Status params start at index 18

// Parameter types (for filtering)
#define CRITICAL_PARAM              0    // Must always be sent
#define NON_CRITICAL_PARAM          1    // Can be compressed
#define NEGLECTED_PARAM             2    // Never sent (debug registers, timers)

// Maximum packet sizes
#define MAX_BINARY_PACKET_SIZE      62   // From timestamped_packet_t.data[62]
#define MAX_COMPRESSED_PAYLOAD_SIZE ((1 + 2) * NUM_OF_STATUS_PARAMS)  // idx(1B) + val(2B) per param


// COMPRESSION CONFIGURATION


// Compression control flags
extern bool compression_enable;           // Global enable/disable for compression
extern bool force_pkt_push_flag;          // Force sending pilot packet

// Pilot packet refresh flags
extern bool send_full_status_pkt_flag;    // Send full uncompressed status packet
extern bool send_full_mode_pkt_flag;      // Send full uncompressed mode packet (not used, mode never compressed)

// Power-on-reset flags to establish initial baseline
extern bool por_update_status_pilot_flag;
extern bool por_update_mode_pilot_flag;

// Force packet timeout (in seconds) - send pilot baseline periodically
extern uint32_t force_pkt_timeout;        // Default: 120 seconds (2 minutes)

// Threshold arrays (loaded from NVS or hardcoded)
extern uint16_t thresholds[NUM_OF_TOTAL_PARAMS];     // Minor thresholds for delta comparison
extern uint16_t min_val[NUM_OF_TOTAL_PARAMS];        // Major thresholds (minimum valid values)
extern uint8_t param_types[NUM_OF_TOTAL_PARAMS];     // Parameter types (CRITICAL/NON_CRITICAL/NEGLECTED)

// Pilot baseline storage (values to compare against)
extern uint16_t pilot_status_params[NUM_OF_STATUS_PARAMS];
extern uint16_t pilot_mode_params[NUM_OF_MODE_PARAMS];

// Current packet parameter storage
extern uint16_t current_status_params[NUM_OF_STATUS_PARAMS];
extern uint16_t current_mode_params[NUM_OF_MODE_PARAMS];

// Timestamp for last forced packet push
extern int64_t last_forced_pkt_time;


// Context passed to compression processing function
typedef struct {
    uint16_t *pilot_params;         // Pointer to pilot baseline array
    uint16_t *current_params;       // Pointer to current params array
    uint16_t *thresholds;           // Pointer to threshold array
    uint16_t *min_val;              // Pointer to minimum value array
    uint8_t *param_types;           // Pointer to parameter type array
    uint16_t param_ptr;             // Starting index in arrays (MODE_PKT_PTR or STATUS_PKT_PTR)
    uint16_t param_count;           // Number of parameters to process
    bool *pilot_update_flag;        // Flag indicating if pilot should be updated
    bool is_full_pkt;               // Flag to send full packet instead of compressed
} CompressionContext;


/*
Loads compression configuration from NVS (or uses hardcoded values).
Sets up thresholds, parameter types, and initializes pilot baselines.
Must be called once during system initialization.
*/
void compression_init(void);

/*
Main compression function for binary packets
Takes a binary packet, applies delta-based compression, and outputs
compressed packet or full packet based on flags and thresholds.
*/
void compress_packet(const uint8_t *packet_in, size_t packet_in_len, uint8_t *packet_out, size_t *packet_out_len);



void process_packet_params(const uint8_t *packet, CompressionContext *context, uint8_t *payload_out, size_t *payload_len);


void check_force_timeout(int64_t current_time_sec);

int16_t abs_s(int16_t number);

uint16_t extract_uint16(const uint8_t *data);

void write_uint16(uint8_t *data, uint16_t value);

void log_compression_stats(void);

#endif