#include "main.h"               // FIRST
#include "packets.h"
#include "tft.h"
#include <string.h>


// Need to calculate the remainder using the poly 0x1021
uint16_t crc16_ccitt(const uint8_t *header, const uint8_t *data, uint32_t len)
{
    uint16_t remainder = CRC_INIT;

    // 1) Feed the 7 header bytes: [version, type, flags, seq_hi, seq_lo, len_hi, len_lo]
    for (uint32_t i = 0; i < CRC_HEADER_LEN; i++) {
        remainder ^= (uint16_t)(header[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (remainder & 0x8000) {
                remainder = (uint16_t)((remainder << 1) ^ CRC_POLY);
            } else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
    }

    // 2) Then feed the payload bytes
    for (uint32_t i = 0; i < len; i++) {
        remainder ^= (uint16_t)(data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (remainder & 0x8000) {
                remainder = (uint16_t)((remainder << 1) ^ CRC_POLY);
            } else {
                remainder = (uint16_t)(remainder << 1);
            }
        }
    }

    return remainder;
}

// ---- Serialize (build) a packet into a buffer ----
// Returns total bytes written, or 0 on error (e.g., buffer too small).
// buf layout after call: SOF0 SOF1 header payload CRC
uint32_t pkt_build(uint8_t* buf, uint32_t buf_size,
                   pkt_data_type_t data_type, uint8_t flags,
                   uint16_t seq, const uint8_t* payload, uint16_t len){
                    if (buf_size < (uint32_t)(11u + len)) return 0;
                    if (!buf) return 0;
                    if (len && !payload) return 0;
                        buf[0] = PACKET_SOF0;                           // Idetifier byte 1
                        buf[1] = PACKET_SOF1;                           // Identifier byte 2
                        buf[2] = PACKET_VERSION;                        // Version Control
                        buf[3] = (uint8_t)data_type;                    // Data type [image,audio,command,acknowledge]
                        buf[4] = flags;                                 // flags
                        buf[5] = (uint8_t)(seq >> 8);                   // High bits for the sequence identifier
                        buf[6] = (uint8_t)seq;                          // Low birs for the sequence identifier
                        buf[7] = (uint8_t)(len >> 8);                   // High bits for length identifier
                        buf[8] = (uint8_t)len;                          // low bits for length identifier
                        memcpy(&buf[9], payload, len);                  // Copy payload into the buffer
                        uint16_t crc = crc16_ccitt(&buf[2], &buf[9], len);   // Calculate CRC from header + payload
                        buf[9+len] = (uint8_t)(crc >> 8);               // High bits for CRC validation
                        buf[10+len] = (uint8_t)crc;                     // Low buts for CRC validation
                        return 11 + len;                                // Return length if successful
                   }

/*
// ---- Parse (from a streaming receiver / ring buffer) ----
// Attempts to find SOF, validate CRC, and output header+payload pointers
// (payload_ptr points into 'frame' memory you pass in).
// Returns total frame size (bytes to consume) on success, 0 if not enough data / bad CRC.
uint32_t pkt_parse(const uint8_t* frame, uint32_t frame_len,
                   pkt_header_t* out_hdr, const uint8_t** out_payload){
                    uint16_t packet_start;
                    int found = 0;
                    for (uint32_t i = 0 ; i < frame_len - 1; ++i){
                        if (frame[i] == PACKET_SOF0){
                            if (frame[i+1] == PACKET_SOF1){
                                packet_start = i + 2;
                                found = 1;
                                break;
                            }
                        }
                    }
                    if (found == 0) return 0;                               // Packet not found
                    out_hdr->version = frame[packet_start];
                    out_hdr->data_type = frame[packet_start+1];
                    out_hdr->flags = frame[packet_start+2];
                    out_hdr->seq = (uint16_t)((frame[packet_start+3] << 8) + frame[packet_start+4]);
                    out_hdr->len = (uint16_t)((frame[packet_start+5] << 8) + frame[packet_start+6]);
                    *out_payload =  &frame[packet_start+7];
                    uint16_t ccr_check = crc16_ccitt(&frame[packet_start], out_hdr->len+7 );
                    uint16_t ccr = (uint16_t)((frame[packet_start+7+out_hdr->len] << 8) + frame[packet_start+8+out_hdr->len]);
                    if (ccr == ccr_check){
                        // bytes from the first SOF through end of CRC:
                        return (packet_start - 2u) + 11u + out_hdr->len;

                    }
                    else {
                        return 0;
                    }
                   }
                   */

uint32_t create_image_data(uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h,
                           const uint8_t* values,
                           uint8_t* data,
                           uint32_t data_buf_size)
{
    uint32_t payload_size = (uint32_t)w * (uint32_t)h * 2u;
    uint32_t total_size = 8u + payload_size;

    if (total_size > data_buf_size) {
        return 0; // not enough space
    }

    data[0] = (uint8_t)(x >> 8);
    data[1] = (uint8_t)x;
    data[2] = (uint8_t)(y >> 8);
    data[3] = (uint8_t)y;
    data[4] = (uint8_t)(w >> 8);
    data[5] = (uint8_t)w;
    data[6] = (uint8_t)(h >> 8);
    data[7] = (uint8_t)h;

    memcpy(&data[8], values, payload_size);
    return total_size;
}


// Returns bytes consumed on success, 0 on error
uint32_t parse_and_apply_image_packet(const uint8_t* data, uint16_t data_len)
{
    if (data_len < 8u) {
        return 0; // not even header
    }

    uint16_t x = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t y = (uint16_t)((data[2] << 8) | data[3]);
    uint16_t w = (uint16_t)((data[4] << 8) | data[5]);
    uint16_t h = (uint16_t)((data[6] << 8) | data[7]);

    uint32_t payload_size = (uint32_t)w * (uint32_t)h * 2u;
    uint32_t total_size = 8u + payload_size;

    if (total_size > data_len) {
        return 0; // incomplete packet
    }

    const uint8_t* data_stream = &data[8];

    // Depending on tft_blit signature you might want:
    // (const uint16_t*) cast if it's RGB565:
    // tft_blit565(x, y, w, h, (const uint16_t*)data_stream);
    tft_blit565(x, y, w, h, data_stream);

    return total_size;
}

/* void process_packets(uint8_t *data, uint16_t len)
{
    // parse header, length, payload, etc. using data[0..len-1]
    pkt_header_t header;
    const uint8_t* payload = NULL;

    // Try to parse one packet out of [data .. data+len)
    uint32_t consumed = pkt_parse((const uint8_t*)data, (uint32_t)len,
                                  &header, &payload);


    if (consumed == 0) {
        // no valid/complete packet yet – you may want to buffer and wait
        return;
    }


	switch(header.data_type){
        case PKT_DATA_IMAGE:
            parse_and_apply_image_packet(payload, header.len);
            break;
        case PKT_DATA_AUDIO:

            break;
        case PKT_DATA_CMD:

            break;
        case PKT_DATA_ACK:

            break;
        default:
            return;
    }

}
*/


