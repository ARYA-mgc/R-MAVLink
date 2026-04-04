#pragma once
#ifndef RELIABLE_PROTOCOL_H
#define RELIABLE_PROTOCOL_H

#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>
#include <array>
#include <chrono>
#include <atomic>

// ─────────────────────────────────────────────
//  R-MAVLink  –  Reliable MAVLink Protocol
//  Version: 1.0.0
// ─────────────────────────────────────────────

namespace rmavlink {

// ── Constants ──────────────────────────────────
static constexpr uint16_t RMAV_MAGIC         = 0xAF42;  // Frame sync word
static constexpr uint8_t  RMAV_VERSION       = 1;
static constexpr uint32_t TIMEOUT_MS         = 100;     // Default ACK timeout (ms)
static constexpr uint8_t  MAX_RETRIES        = 4;       // Max retransmission attempts
static constexpr uint8_t  WINDOW_SIZE        = 8;       // Sliding window size
static constexpr uint16_t MAX_PAYLOAD_SIZE   = 280;     // Bytes (MAVLink max = 263 + overhead)
static constexpr uint16_t MAX_SEQ_ID         = 65535;   // uint16_t rollover

// ── Message Types ──────────────────────────────
enum class MsgType : uint8_t {
    DATA  = 0x01,   // Data packet carrying MAVLink payload
    ACK   = 0x02,   // Positive acknowledgement
    NACK  = 0x03,   // Negative acknowledgement (request resend)
    PING  = 0x04,   // Heartbeat / link check
    PONG  = 0x05,   // Heartbeat reply
    RESET = 0x06,   // Reset sequence numbers
};

// ── Packet States ──────────────────────────────
enum class PacketState : uint8_t {
    PENDING    = 0,  // Waiting to be sent
    SENT       = 1,  // Sent, awaiting ACK
    ACKED      = 2,  // Acknowledged — done
    NACKED     = 3,  // NACK received — retransmit
    LOST       = 4,  // Exceeded MAX_RETRIES
};

// ── Error Codes ────────────────────────────────
enum class Error : int {
    OK             =  0,
    TIMEOUT        = -1,
    MAX_RETRIES    = -2,
    BUFFER_FULL    = -3,
    INVALID_FRAME  = -4,
    CRC_MISMATCH   = -5,
    OUT_OF_ORDER   = -6,
    DUPLICATE      = -7,
    NULL_TRANSPORT = -8,
};

// ─────────────────────────────────────────────
//  ReliableHeader  (8 bytes, little-endian)
// ─────────────────────────────────────────────
#pragma pack(push, 1)
struct ReliableHeader {
    uint16_t magic;         // 0xAF42 — frame sync
    uint16_t seq_id;        // Packet sequence number (wraps at 65535)
    uint8_t  msg_type;      // MsgType enum
    uint16_t timestamp;     // Milliseconds mod 65536 (sender-local clock)
    uint8_t  payload_len;   // Length of MAVLink payload (0 for ACK/NACK)
    uint16_t header_crc;    // CRC-16 of header fields (excluding this field)
};

struct ReliableFrame {
    ReliableHeader header;
    uint8_t        payload[MAX_PAYLOAD_SIZE];
    uint16_t       frame_crc;   // CRC-16 of header + payload
};
#pragma pack(pop)

static constexpr size_t HEADER_SIZE = sizeof(ReliableHeader);
static constexpr size_t MIN_FRAME_SIZE = HEADER_SIZE + sizeof(uint16_t); // header + frame_crc

// ─────────────────────────────────────────────
//  Tracked Packet  (sender-side window entry)
// ─────────────────────────────────────────────
struct TrackedPacket {
    ReliableFrame frame;
    PacketState   state          = PacketState::PENDING;
    uint8_t       retry_count    = 0;
    uint64_t      send_time_ms   = 0;  // Absolute send timestamp
    size_t        frame_size     = 0;  // Total bytes in frame
};

// ─────────────────────────────────────────────
//  Transport Interface  (implement for UART / UDP)
// ─────────────────────────────────────────────
class ITransport {
public:
    virtual ~ITransport() = default;

    /// Send exactly `len` bytes. Returns bytes sent, or <0 on error.
    virtual int send(const uint8_t* data, size_t len) = 0;

    /// Receive up to `max_len` bytes. Returns bytes received (0 = nothing yet), or <0 on error.
    virtual int recv(uint8_t* buf, size_t max_len) = 0;

    /// Current monotonic time in milliseconds.
    virtual uint64_t now_ms() = 0;

    /// True if the link is connected/ready.
    virtual bool is_connected() = 0;
};

// ─────────────────────────────────────────────
//  Statistics
// ─────────────────────────────────────────────
struct Stats {
    uint64_t packets_sent        = 0;
    uint64_t packets_received    = 0;
    uint64_t acks_sent           = 0;
    uint64_t acks_received       = 0;
    uint64_t nacks_sent          = 0;
    uint64_t nacks_received      = 0;
    uint64_t retransmissions     = 0;
    uint64_t packets_lost        = 0;   // Exceeded MAX_RETRIES
    uint64_t duplicates_dropped  = 0;
    uint64_t crc_errors          = 0;

    float delivery_rate() const {
        if (packets_sent == 0) return 0.0f;
        return 100.0f * (float)(packets_sent - packets_lost) / (float)packets_sent;
    }
};

} // namespace rmavlink

#endif // RELIABLE_PROTOCOL_H
