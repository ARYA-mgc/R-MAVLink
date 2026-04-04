// mavlink_parser.cpp
// Minimal MAVLink v2 framing layer.
// Wraps / unwraps MAVLink packets to feed into R-MAVLink reliable transport.
//
// If you have the official mavlink/c_library_v2 headers, define
//   HAVE_MAVLINK_HEADERS
// and include them above. Otherwise this file provides a lightweight
// standalone framing implementation compatible with MAVLink v2.

#include "../include/reliable_protocol.h"
#include <cstring>
#include <cstdio>

// ── Optional: use official headers ─────────────────────────────────────────
#ifdef HAVE_MAVLINK_HEADERS
  #include "mavlink/common/mavlink.h"
#else
  // ── Minimal MAVLink v2 constants ──────────────────────────────────────────
  static constexpr uint8_t MAVLINK2_STX        = 0xFD;
  static constexpr uint8_t MAVLINK2_HDR_LEN    = 10;  // Bytes before payload
  static constexpr uint8_t MAVLINK2_CHKSUM_LEN = 2;
  static constexpr size_t  MAVLINK2_MAX_PACKET  = 280; // STX + 9 + 255 + 2 + (13 sig)

  #pragma pack(push, 1)
  struct Mavlink2Header {
      uint8_t  stx;          // 0xFD
      uint8_t  len;          // Payload length (0–255)
      uint8_t  incompat_flags;
      uint8_t  compat_flags;
      uint8_t  seq;
      uint8_t  sysid;
      uint8_t  compid;
      uint32_t msgid : 24;   // 24-bit message ID
  };
  #pragma pack(pop)
#endif

namespace rmavlink {

// ─────────────────────────────────────────────
//  MavlinkParser
//
//  Provides two services:
//    1. build_packet()  – pack a MAVLink v2 message into a byte buffer
//    2. parse()         – extract MAVLink messages from a raw byte stream
// ─────────────────────────────────────────────
class MavlinkParser {
public:
    MavlinkParser(uint8_t sysid = 1, uint8_t compid = 1)
        : sysid_(sysid), compid_(compid), seq_(0) {}

    // ── Build a raw MAVLink v2 packet ──────────────────────────────────────

    /// Packs a MAVLink v2 DATA frame into `out_buf`.
    /// `msgid`    : 24-bit message ID (e.g., 0 = HEARTBEAT)
    /// `payload`  : raw payload bytes
    /// `len`      : payload length (0–255)
    /// Returns total packet length, or 0 on error.
    size_t build_packet(uint8_t* out_buf, size_t out_size,
                        uint32_t msgid, const uint8_t* payload, uint8_t len)
    {
        size_t total = MAVLINK2_HDR_LEN + len + MAVLINK2_CHKSUM_LEN;
        if (out_size < total) return 0;

#ifndef HAVE_MAVLINK_HEADERS
        out_buf[0] = MAVLINK2_STX;
        out_buf[1] = len;
        out_buf[2] = 0;        // incompat_flags
        out_buf[3] = 0;        // compat_flags
        out_buf[4] = seq_++;
        out_buf[5] = sysid_;
        out_buf[6] = compid_;
        out_buf[7] = (msgid)       & 0xFF;
        out_buf[8] = (msgid >> 8)  & 0xFF;
        out_buf[9] = (msgid >> 16) & 0xFF;
        if (len > 0) std::memcpy(out_buf + 10, payload, len);

        uint16_t crc = x25_crc(out_buf + 1, MAVLINK2_HDR_LEN - 1 + len);
        // Add CRC extra byte (0x50 for HEARTBEAT msgid=0; use 0 for generic)
        crc = x25_accumulate(crc, crc_extra_for(msgid));
        out_buf[10 + len] = crc & 0xFF;
        out_buf[11 + len] = (crc >> 8) & 0xFF;
#endif
        return total;
    }

    // ── Parse incoming raw bytes ───────────────────────────────────────────

    /// Feed raw bytes; calls `on_packet` for each complete MAVLink frame.
    /// on_packet(raw_frame_ptr, total_len, msgid)
    using PacketCallback = std::function<void(const uint8_t*, size_t, uint32_t)>;

    void feed_bytes(const uint8_t* data, size_t len, PacketCallback on_packet) {
        for (size_t i = 0; i < len; ++i) {
            parse_byte(data[i], on_packet);
        }
    }

private:
    uint8_t sysid_, compid_;
    uint8_t seq_;

    // Simple state-machine parser
    enum class ParseState { IDLE, GOT_STX, IN_HEADER, IN_PAYLOAD, IN_CRC };
    ParseState  pstate_  = ParseState::IDLE;
    uint8_t     pbuf_[MAVLINK2_MAX_PACKET]{};
    size_t      pbuf_idx_ = 0;
    uint8_t     expected_payload_len_ = 0;

    void parse_byte(uint8_t b, PacketCallback& cb) {
        switch (pstate_) {
            case ParseState::IDLE:
                if (b == MAVLINK2_STX) {
                    pbuf_idx_ = 0;
                    pbuf_[pbuf_idx_++] = b;
                    pstate_ = ParseState::GOT_STX;
                }
                break;

            case ParseState::GOT_STX:
                expected_payload_len_ = b;  // len field
                pbuf_[pbuf_idx_++] = b;
                pstate_ = ParseState::IN_HEADER;
                break;

            case ParseState::IN_HEADER:
                pbuf_[pbuf_idx_++] = b;
                if (pbuf_idx_ == MAVLINK2_HDR_LEN) {
                    pstate_ = (expected_payload_len_ > 0) ? ParseState::IN_PAYLOAD
                                                          : ParseState::IN_CRC;
                }
                break;

            case ParseState::IN_PAYLOAD:
                pbuf_[pbuf_idx_++] = b;
                if (pbuf_idx_ == MAVLINK2_HDR_LEN + expected_payload_len_)
                    pstate_ = ParseState::IN_CRC;
                break;

            case ParseState::IN_CRC:
                pbuf_[pbuf_idx_++] = b;
                if (pbuf_idx_ == MAVLINK2_HDR_LEN + expected_payload_len_ + MAVLINK2_CHKSUM_LEN) {
                    // Extract msgid (bytes 7–9, little-endian 24-bit)
                    uint32_t msgid = pbuf_[7] | ((uint32_t)pbuf_[8] << 8) | ((uint32_t)pbuf_[9] << 16);
                    if (cb) cb(pbuf_, pbuf_idx_, msgid);
                    pstate_ = ParseState::IDLE;
                }
                break;
        }
    }

    // ── X.25 CRC (MAVLink standard) ───────────────────────────────────────
    static uint16_t x25_crc(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        while (len--) crc = x25_accumulate(crc, *data++);
        return crc;
    }
    static uint16_t x25_accumulate(uint16_t crc, uint8_t b) {
        uint8_t tmp = b ^ (crc & 0xFF);
        tmp ^= (tmp << 4);
        return (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4);
    }

    /// CRC extra bytes for common message IDs (add more as needed).
    static uint8_t crc_extra_for(uint32_t msgid) {
        switch (msgid) {
            case 0:   return 0x50;  // HEARTBEAT
            case 30:  return 0x39;  // ATTITUDE
            case 33:  return 0x61;  // GLOBAL_POSITION_INT
            case 74:  return 0x14;  // VFR_HUD
            case 245: return 0x8B;  // EXTENDED_SYS_STATE
            default:  return 0x00;
        }
    }
};

} // namespace rmavlink
