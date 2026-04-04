#pragma once
#ifndef PACKET_MANAGER_H
#define PACKET_MANAGER_H

#include "reliable_protocol.h"
#include "crc_utils.h"
#include <array>
#include <cstring>
#include <cstdio>

namespace rmavlink {

// ─────────────────────────────────────────────
//  PacketManager
//
//  Handles:
//   • Frame building & parsing
//   • Sliding-window send queue
//   • ACK / NACK processing
//   • Retransmission on timeout
//   • Duplicate detection on receive side
// ─────────────────────────────────────────────
class PacketManager {
public:
    explicit PacketManager(ITransport* transport)
        : transport_(transport), next_seq_(1), last_recv_seq_(0),
          window_base_(0), window_head_(0)
    {
        window_.fill(TrackedPacket{});
    }

    // ── Sending ────────────────────────────────

    /// Enqueue a MAVLink payload for reliable delivery.
    /// Returns Error::OK or Error::BUFFER_FULL.
    Error send(const uint8_t* mavlink_data, size_t len) {
        if (transport_ == nullptr) return Error::NULL_TRANSPORT;
        if (window_used() >= WINDOW_SIZE) return Error::BUFFER_FULL;
        if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;

        TrackedPacket& slot = window_[window_head_ % WINDOW_SIZE];
        build_frame(slot.frame, next_seq_, MsgType::DATA, mavlink_data, (uint8_t)len);
        slot.state        = PacketState::PENDING;
        slot.retry_count  = 0;
        slot.send_time_ms = 0;
        slot.frame_size   = HEADER_SIZE + len + sizeof(uint16_t);

        ++window_head_;
        ++next_seq_;
        if (next_seq_ == 0) next_seq_ = 1;  // Avoid seq=0

        stats_.packets_sent++;
        return Error::OK;
    }

    /// Must be called regularly (e.g., every 1–5 ms) to drive the protocol.
    void tick() {
        if (transport_ == nullptr) return;

        // 1. Transmit or retransmit pending/timed-out packets
        for (size_t i = window_base_; i < window_head_; ++i) {
            TrackedPacket& pkt = window_[i % WINDOW_SIZE];

            if (pkt.state == PacketState::ACKED) continue;

            uint64_t now = transport_->now_ms();

            bool should_send = false;
            if (pkt.state == PacketState::PENDING) {
                should_send = true;
            } else if (pkt.state == PacketState::SENT || pkt.state == PacketState::NACKED) {
                if (now - pkt.send_time_ms > TIMEOUT_MS) {
                    if (pkt.retry_count >= MAX_RETRIES) {
                        pkt.state = PacketState::LOST;
                        stats_.packets_lost++;
                        // Slide base past lost packet to unblock window
                        if (i == window_base_) advance_base();
                        continue;
                    }
                    should_send = true;
                    stats_.retransmissions++;
                }
            }

            if (should_send) {
                // Update timestamp in frame before sending
                uint16_t ts = (uint16_t)(transport_->now_ms() & 0xFFFF);
                pkt.frame.header.timestamp = ts;
                recompute_crcs(pkt.frame, pkt.frame_size);

                // Serialize into a flat wire buffer: [header][payload][frame_crc]
                size_t payload_len = pkt.frame.header.payload_len;
                uint8_t wire[HEADER_SIZE + MAX_PAYLOAD_SIZE + sizeof(uint16_t)];
                std::memcpy(wire, &pkt.frame.header, HEADER_SIZE);
                std::memcpy(wire + HEADER_SIZE, pkt.frame.payload, payload_len);
                std::memcpy(wire + HEADER_SIZE + payload_len, &pkt.frame.frame_crc, sizeof(uint16_t));
                size_t wire_len = HEADER_SIZE + payload_len + sizeof(uint16_t);

                transport_->send(wire, wire_len);
                pkt.state        = PacketState::SENT;
                pkt.send_time_ms = transport_->now_ms();
                pkt.retry_count++;
            }
        }

        // 2. Drain all available incoming frames (ACK, NACK, DATA)
        uint8_t buf[sizeof(ReliableFrame)];
        int n;
        while ((n = transport_->recv(buf, sizeof(buf))) > 0) {
            process_incoming(buf, (size_t)n);
        }
    }

    /// Process a raw byte buffer that was received (call instead of tick() if
    /// you handle receive polling externally).
    void process_incoming(const uint8_t* buf, size_t len) {
        if (len < MIN_FRAME_SIZE) return;

        // Parse header
        ReliableFrame frame;
        std::memcpy(&frame.header, buf, HEADER_SIZE);

        if (frame.header.magic != RMAV_MAGIC) {
            stats_.crc_errors++;
            return;
        }

        // Validate header CRC
        size_t crc_field_offset = offsetof(ReliableHeader, header_crc);
        uint16_t expected_hcrc = compute_header_crc(buf, HEADER_SIZE, crc_field_offset);
        if (frame.header.header_crc != expected_hcrc) {
            stats_.crc_errors++;
            return;
        }

        size_t payload_len = frame.header.payload_len;
        size_t total       = HEADER_SIZE + payload_len + sizeof(uint16_t);
        if (len < total) return;

        std::memcpy(frame.payload, buf + HEADER_SIZE, payload_len);
        uint16_t recv_fcrc;
        std::memcpy(&recv_fcrc, buf + HEADER_SIZE + payload_len, sizeof(uint16_t));

        // Validate frame CRC
        uint16_t computed_fcrc = crc16(buf, HEADER_SIZE + payload_len);
        if (recv_fcrc != computed_fcrc) {
            stats_.crc_errors++;
            // Send NACK so sender retransmits
            send_nack(frame.header.seq_id);
            return;
        }

        auto type = static_cast<MsgType>(frame.header.msg_type);

        switch (type) {
            case MsgType::ACK:
                handle_ack(frame.header.seq_id);
                break;
            case MsgType::NACK:
                handle_nack(frame.header.seq_id);
                break;
            case MsgType::DATA:
                handle_data(frame);
                break;
            case MsgType::PING:
                send_pong(frame.header.seq_id);
                break;
            default:
                break;
        }
    }

    // ── Callbacks ──────────────────────────────

    /// Called when a complete, validated DATA packet is received.
    std::function<void(const uint8_t* payload, size_t len, uint16_t seq)> on_data_received;

    /// Called when a packet is permanently lost (exceeded MAX_RETRIES).
    std::function<void(uint16_t seq)> on_packet_lost;

    // ── Accessors ──────────────────────────────
    const Stats& stats() const { return stats_; }
    void reset_stats() { stats_ = {}; }

    size_t window_used() const {
        return (window_head_ >= window_base_) ? (window_head_ - window_base_) : 0;
    }

    bool window_full() const { return window_used() >= WINDOW_SIZE; }

private:
    ITransport*                             transport_;
    uint16_t                                next_seq_;
    uint16_t                                last_recv_seq_;
    size_t                                  window_base_;
    size_t                                  window_head_;
    std::array<TrackedPacket, WINDOW_SIZE>  window_;
    Stats                                   stats_;

    // ── Frame Building ─────────────────────────

    void build_frame(ReliableFrame& frame, uint16_t seq, MsgType type,
                     const uint8_t* payload, uint8_t payload_len)
    {
        frame.header.magic       = RMAV_MAGIC;
        frame.header.seq_id      = seq;
        frame.header.msg_type    = static_cast<uint8_t>(type);
        frame.header.timestamp   = transport_ ? (uint16_t)(transport_->now_ms() & 0xFFFF) : 0;
        frame.header.payload_len = payload_len;

        if (payload && payload_len > 0)
            std::memcpy(frame.payload, payload, payload_len);

        // Header CRC (excluding header_crc field)
        size_t crc_off = offsetof(ReliableHeader, header_crc);
        frame.header.header_crc = compute_header_crc(
            reinterpret_cast<uint8_t*>(&frame.header), HEADER_SIZE, crc_off);

        // Frame CRC (header + payload)
        uint8_t tmp[HEADER_SIZE + MAX_PAYLOAD_SIZE];
        std::memcpy(tmp, &frame.header, HEADER_SIZE);
        if (payload_len > 0) std::memcpy(tmp + HEADER_SIZE, frame.payload, payload_len);
        frame.frame_crc = crc16(tmp, HEADER_SIZE + payload_len);
    }

    void recompute_crcs(ReliableFrame& frame, size_t frame_size) {
        size_t payload_len = frame.header.payload_len;

        size_t crc_off = offsetof(ReliableHeader, header_crc);
        frame.header.header_crc = compute_header_crc(
            reinterpret_cast<uint8_t*>(&frame.header), HEADER_SIZE, crc_off);

        uint8_t tmp[HEADER_SIZE + MAX_PAYLOAD_SIZE];
        std::memcpy(tmp, &frame.header, HEADER_SIZE);
        std::memcpy(tmp + HEADER_SIZE, frame.payload, payload_len);
        frame.frame_crc = crc16(tmp, HEADER_SIZE + payload_len);
    }

    // ── ACK / NACK helpers ─────────────────────

    void send_ack(uint16_t seq) {
        ReliableFrame ack{};
        build_frame(ack, seq, MsgType::ACK, nullptr, 0);
        uint8_t buf[HEADER_SIZE + sizeof(uint16_t)];
        std::memcpy(buf, &ack.header, HEADER_SIZE);
        std::memcpy(buf + HEADER_SIZE, &ack.frame_crc, sizeof(uint16_t));
        transport_->send(buf, sizeof(buf));
        stats_.acks_sent++;
    }

    void send_nack(uint16_t seq) {
        ReliableFrame nack{};
        build_frame(nack, seq, MsgType::NACK, nullptr, 0);
        uint8_t buf[HEADER_SIZE + sizeof(uint16_t)];
        std::memcpy(buf, &nack.header, HEADER_SIZE);
        std::memcpy(buf + HEADER_SIZE, &nack.frame_crc, sizeof(uint16_t));
        transport_->send(buf, sizeof(buf));
        stats_.nacks_sent++;
    }

    void send_pong(uint16_t seq) {
        ReliableFrame pong{};
        build_frame(pong, seq, MsgType::PONG, nullptr, 0);
        uint8_t buf[HEADER_SIZE + sizeof(uint16_t)];
        std::memcpy(buf, &pong.header, HEADER_SIZE);
        std::memcpy(buf + HEADER_SIZE, &pong.frame_crc, sizeof(uint16_t));
        transport_->send(buf, sizeof(buf));
    }

    // ── Event handlers ─────────────────────────

    void handle_ack(uint16_t seq) {
        stats_.acks_received++;
        for (size_t i = window_base_; i < window_head_; ++i) {
            TrackedPacket& pkt = window_[i % WINDOW_SIZE];
            if (pkt.frame.header.seq_id == seq) {
                pkt.state = PacketState::ACKED;
                break;
            }
        }
        // Slide window base over consecutive ACKed packets
        advance_base();
    }

    void handle_nack(uint16_t seq) {
        stats_.nacks_received++;
        for (size_t i = window_base_; i < window_head_; ++i) {
            TrackedPacket& pkt = window_[i % WINDOW_SIZE];
            if (pkt.frame.header.seq_id == seq) {
                pkt.state        = PacketState::NACKED;
                pkt.send_time_ms = 0;  // Force immediate retransmit
                break;
            }
        }
    }

    void handle_data(const ReliableFrame& frame) {
        stats_.packets_received++;
        uint16_t seq = frame.header.seq_id;

        // Duplicate detection
        if (seq_le(seq, last_recv_seq_)) {
            stats_.duplicates_dropped++;
            send_ack(seq);  // Re-ACK so sender knows we have it
            return;
        }

        last_recv_seq_ = seq;
        send_ack(seq);

        if (on_data_received)
            on_data_received(frame.payload, frame.header.payload_len, seq);
    }

    void advance_base() {
        while (window_base_ < window_head_) {
            TrackedPacket& pkt = window_[window_base_ % WINDOW_SIZE];
            if (pkt.state == PacketState::ACKED || pkt.state == PacketState::LOST) {
                if (pkt.state == PacketState::LOST && on_packet_lost)
                    on_packet_lost(pkt.frame.header.seq_id);
                ++window_base_;
            } else {
                break;
            }
        }
    }

    /// Sequence number less-than-or-equal with wrap-around.
    static bool seq_le(uint16_t a, uint16_t b) {
        // Works for up to half-range difference
        return ((b - a) & 0x8000) == 0 && a != b + 1;
    }
};

} // namespace rmavlink

#endif // PACKET_MANAGER_H
