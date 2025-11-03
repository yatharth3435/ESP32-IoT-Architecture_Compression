from flask import Flask, request
from datetime import datetime
import base64
import re

app = Flask(__name__)

@app.route('/esp32/data', methods=['POST'])
def receive_data():
    """
    Handles POST requests from the ESP32.
    Receives Base64-encoded batches of timestamped binary packets.
    
    FIXED: Uses fixed packet lengths instead of scanning for CRLF delimiter
    to avoid false positives when 0x0D 0x0A appears in packet data.
    
    Packet format: #T[8-hex-epoch]#[type][data]
    """
    raw_base64_data = request.get_data()
    
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    if raw_base64_data:
        print(f"--- {timestamp} ---")
        print(f"Received {len(raw_base64_data)} bytes of Base64 data.")

        try:
            # Decode from Base64
            decoded_data = base64.b64decode(raw_base64_data)
            print(f"Decoded to {len(decoded_data)} bytes of binary data.")
            
            # Save the raw Base64 data
            with open('received_base64.log', 'ab') as f:
                f.write(raw_base64_data + b'\n')
            
            # Save the decoded binary data
            with open('received_decoded.bin', 'ab') as f:
                f.write(decoded_data)
            
            # --- FIXED PACKET PARSING LOGIC ---
            packets_found = 0
            
            with open('received_packets_parsed.txt', 'a') as f_parsed:
                i = 0
                
                while i < len(decoded_data):
                    # Look for packet start: #T or #
                    if decoded_data[i] != 0x23:  # Not '#'
                        i += 1
                        continue
                    
                    has_timestamp = False
                    timestamp_hex = None
                    
                    # Check if this is #T (timestamp header)
                    if i + 1 < len(decoded_data) and decoded_data[i + 1] == 0x54:  # 'T'
                        # This is a timestamp header: #T[8-hex]
                        if i + 10 <= len(decoded_data):
                            try:
                                # Extract timestamp (8 ASCII hex characters)
                                timestamp_bytes = decoded_data[i + 2:i + 10]
                                timestamp_hex = timestamp_bytes.decode('ascii')
                                int(timestamp_hex, 16)  # Verify it's valid hex
                                has_timestamp = True
                                i += 10  # Skip past #T[8-hex]
                            except:
                                # Not a valid timestamp, skip this '#'
                                i += 1
                                continue
                        else:
                            # Not enough bytes for timestamp
                            break
                    
                    # Now we should be at # (packet marker)
                    if i >= len(decoded_data) or decoded_data[i] != 0x23:
                        i += 1
                        continue
                    
                    # Check packet type
                    if i + 1 >= len(decoded_data):
                        break
                    
                    packet_type_byte = decoded_data[i + 1]
                    if packet_type_byte not in [0x73, 0x6d, 0x63]:  # 's', 'm', 'c'
                        i += 1
                        continue
                    
                    packet_type = chr(packet_type_byte)
                    
                    # Determine packet length based on type (fixed lengths for binary packets)
                    # These are the sizes AFTER binary conversion on ESP32 (excluding CRLF)
                    if packet_type == 's':
                        packet_length = 61  # #s + 60 bytes binary data
                    elif packet_type == 'm':
                        packet_length = 45  # #m + 44 bytes binary data
                    elif packet_type == 'c':
                        packet_length = 63  # #c + 62 bytes binary data
                    else:
                        i += 1
                        continue
                    
                    # Check if we have enough bytes for complete packet
                    if i + packet_length > len(decoded_data):
                        print(f"  WARNING: Not enough bytes for packet type '{packet_type}'. "
                              f"Expected {packet_length}, have {len(decoded_data) - i}. Stopping.")
                        break
                    
                    # Extract the complete packet (fixed length, no CRLF scanning needed)
                    packet_bytes = decoded_data[i:i + packet_length]
                    
                    if len(packet_bytes) < 2:
                        i += packet_length
                        continue
                    
                    packets_found += 1
                    
                    # Convert to hex string (skip #[type] prefix)
                    packet_hex = packet_bytes.hex()
                    payload_hex = packet_hex[4:]  # Skip "23" + type byte
                    
                    if has_timestamp:
                        print(f"  Packet {packets_found}: Type='{packet_type}', "
                              f"Timestamp=0x{timestamp_hex}, "
                              f"Size={len(packet_bytes)} bytes")
                        f_parsed.write(f"#T{timestamp_hex}#{packet_type}{payload_hex}\n")
                    else:
                        print(f"  Packet {packets_found}: Type='{packet_type}', "
                              f"Timestamp=NONE (pre-NTP), "
                              f"Size={len(packet_bytes)} bytes")
                        f_parsed.write(f"#{packet_type}{payload_hex}\n")
                    
                    # Move to next packet (jump exactly packet_length bytes)
                    i += packet_length
            
            print(f"Total packets in this batch: {packets_found}")
            print("Data saved to:")
            print("  - received_base64.log (Base64 format)")
            print("  - received_decoded.bin (binary format)")
            print("  - received_packets_parsed.txt (parsed hex format)")
            print("-" * 40 + "\n")

        except Exception as e:
            print(f"Error processing data: {e}")
            import traceback
            traceback.print_exc()
        
        return 'Data received successfully', 200
    else:
        print(f"{timestamp} - Received an empty request.")
        return 'No data received', 400

if __name__ == '__main__':
    print("Starting Flask server...")
    print("Listening for ESP32 data on http://0.0.0.0:5000/esp32/data")
    print("\nOutput files:")
    print("  - received_base64.log: Raw Base64 data")
    print("  - received_decoded.bin: Decoded binary data")
    print("  - received_packets_parsed.txt: Parsed packets in hex format")
    print("\nPacket format expected:")
    print("  - WITH timestamp: #T[8-hex-epoch]#[type][data]")
    print("  - WITHOUT timestamp: #[type][data]")
    print("\nValid packet types: s, m, c")
    print("\nFIXED: Now uses fixed packet lengths (s=62, m=46, c=64 bytes)")
    print("       instead of scanning for CRLF to avoid 0x0D0A false positives")
    app.run(host='0.0.0.0', port=5000, debug=True)