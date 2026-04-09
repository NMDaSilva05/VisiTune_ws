#pragma once
#include <stdint.h>

// Framing Bytes
#define PACKET_SOF0 0xA5
#define PACKET_SOF1 0x5A   
// These 2 bytes are meant to indicate the start of a packet

//Protocol Version
#define PACKET_VERSION 0x01u            // Version 1

// CRC definitions
#define CRC_POLY 0x1021                 // Divisor for validation
#define CRC_INIT 0xFFFF
#define CRC_HEADER_LEN 7   // version..len (bytes 2..8 of the packet)


// Buffer Maximum Size
#define OVERHEAD 11
#define MAX_SIZE 0xFFFFu                // maximum buffer size 4095
#define MAX_DATA_SIZE MAX_SIZE - OVERHEAD

// Data Typesthere
typedef enum {
    PKT_DATA_IMAGE = 0x01,             // Image data
    PKT_DATA_AUDIO = 0x02,             // audio data
    PKT_DATA_CMD   = 0x03,              // commands?
    PKT_DATA_ACK   = 0x04,              // acknowledgement by receiver
	PKT_DATA_TEXT  = 0x05
} pkt_data_type_t;

typedef enum {
    TEXT_FIELD_TITLE  = 0x01,
    TEXT_FIELD_ARTIST = 0x02,
    TEXT_FIELD_ALBUM  = 0x03,
    TEXT_FIELD_OTHER  = 0x04,
} text_field_t;


typedef enum {
	CTRL_CMD_AUDIO_STOP = 0x01
} ctrl_cmd_t;



// Packet Flags
// ---- Flags bitfield (combine as needed) ----
// Bit 0: last chunk in a stream
// Bit 1: compressed payload
// Bit 2: requires ACK
// Bit 3: is retransmission
// Bits 4-7: reserved for future 
enum {
    PKT_FLAG_LAST       = 1u << 0,
    PKT_FLAG_COMPRESSED = 1u << 1,
    PKT_FLAG_NEED_ACK   = 1u << 2,
    PKT_FLAG_RETX       = 1u << 3,
};





// Packet header structure
    // __attribute__((packed)) removes any padding bits to keep the data tight
typedef struct __attribute__((packed)) {
    uint8_t  version;          // = PACKET_VERSION
    uint8_t  data_type;        // pkt_data_type_t
    uint8_t  flags;            // bitfield
    uint16_t seq;              // sequence number (big-endian on wire)
    uint16_t len;              // payload length in bytes (big-endian on wire)
} pkt_header_t;



// ---- CRC16-CCITT (poly 0x1021, init 0xFFFF) ----
// The crc16 is meant to be the remainder when dividing the buffer by the poly
// The receiver should recalculate based on the received data
// If the calculated is different than the received, then we know
// that the packet got corrupted and we need to request a resend
uint16_t crc16_ccitt(const uint8_t* header, const uint8_t* data, uint32_t len);

// ---- Serialize (build) a packet into a buffer ----
// Returns total bytes written, or 0 on error (e.g., buffer too small).
// buf layout after call: SOF0 SOF1 header payload CRC
uint32_t pkt_build(uint8_t* buf, uint32_t buf_size,
                   pkt_data_type_t data_type, uint8_t flags,
                   uint16_t seq, const uint8_t* payload, uint16_t len);

// ---- Parse (from a streaming receiver / ring buffer) ----
// Attempts to find SOF, validate CRC, and output header+payload pointers
// (payload_ptr points into 'frame' memory you pass in).
// Returns total frame size (bytes to consume) on success, 0 if not enough data / bad CRC.
uint32_t pkt_parse(const uint8_t* frame, uint32_t frame_len,
                   pkt_header_t* out_hdr, const uint8_t** out_payload);


// Takes a given x, y, w, h and rgb value array and concatenates into 1 data stream
uint32_t create_image_data(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t* values, uint8_t* data,  uint32_t data_buf_size);


// Takes the data stream from an image packet and parses out the locations
// Then calls tft_blit to pass the data through to the display
uint32_t parse_and_apply_image_packet(const uint8_t* data, uint16_t data_len);

// Take a UART Reception and process the packet

void process_packets(uint8_t *data, uint16_t len);
    



