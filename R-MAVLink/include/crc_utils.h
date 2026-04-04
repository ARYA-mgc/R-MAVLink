#pragma once
#ifndef CRC_UTILS_H
#define CRC_UTILS_H

#include <cstdint>
#include <cstddef>

namespace rmavlink {

// ─────────────────────────────────────────────
//  CRC-16/CCITT-FALSE
//  Poly: 0x1021  Init: 0xFFFF  RefIn: false  RefOut: false
// ─────────────────────────────────────────────
inline uint16_t crc16(const uint8_t* data, size_t len, uint16_t init = 0xFFFF) {
    uint16_t crc = init;
    while (len--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/// Compute CRC over header fields only (excluding header_crc field itself).
inline uint16_t compute_header_crc(const uint8_t* header_bytes, size_t header_size,
                                    size_t crc_field_offset)
{
    uint16_t crc = 0xFFFF;
    // CRC over bytes before the CRC field
    crc = crc16(header_bytes, crc_field_offset, crc);
    // CRC over bytes after the CRC field (if any)
    size_t after_offset = crc_field_offset + sizeof(uint16_t);
    if (after_offset < header_size)
        crc = crc16(header_bytes + after_offset, header_size - after_offset, crc);
    return crc;
}

} // namespace rmavlink

#endif // CRC_UTILS_H
