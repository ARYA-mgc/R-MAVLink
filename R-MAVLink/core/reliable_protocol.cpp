// reliable_protocol.cpp
// Top-level R-MAVLink engine — integrates PacketManager + transport selection.

#include "../include/reliable_protocol.h"
#include "../include/packet_manager.h"
#include <cstdio>
#include <cstring>

// ─────────────────────────────────────────────────────────────────────────────
//  ReliableProtocol  –  Public API
//
//  Usage:
//    UdpTransport transport("127.0.0.1", 14550, 14551);
//    ReliableProtocol proto(&transport);
//    proto.on_receive = [](const uint8_t* data, size_t len, uint16_t seq) { ... };
//
//    // Main loop:
//    while (running) {
//        proto.send(mavlink_buf, mavlink_len);
//        proto.tick();
//    }
// ─────────────────────────────────────────────────────────────────────────────

namespace rmavlink {

class ReliableProtocol {
public:
    explicit ReliableProtocol(ITransport* transport)
        : manager_(transport)
    {
        manager_.on_data_received = [this](const uint8_t* p, size_t l, uint16_t s) {
            if (on_receive) on_receive(p, l, s);
        };
        manager_.on_packet_lost = [this](uint16_t s) {
            if (on_packet_lost) on_packet_lost(s);
        };
    }

    // ── Public API ──────────────────────────────

    /// Queue a MAVLink message for reliable delivery.
    /// Blocks (busy-waits with tick()) if the send window is full.
    Error send(const uint8_t* data, size_t len) {
        Error err;
        while ((err = manager_.send(data, len)) == Error::BUFFER_FULL) {
            manager_.tick();
        }
        return err;
    }

    /// Drive retransmissions and receive processing. Call as fast as possible.
    void tick() { manager_.tick(); }

    /// Feed a raw received buffer (use when transport is polled externally).
    void feed(const uint8_t* buf, size_t len) {
        manager_.process_incoming(buf, len);
    }

    // ── Callbacks ──────────────────────────────
    std::function<void(const uint8_t* payload, size_t len, uint16_t seq)> on_receive;
    std::function<void(uint16_t seq)> on_packet_lost;

    // ── Stats ──────────────────────────────────
    const Stats& stats() const { return manager_.stats(); }
    void print_stats() const {
        const Stats& s = manager_.stats();
        printf("\n=== R-MAVLink Statistics ===\n");
        printf("  Packets sent:          %llu\n", (unsigned long long)s.packets_sent);
        printf("  Packets received:      %llu\n", (unsigned long long)s.packets_received);
        printf("  ACKs sent:             %llu\n", (unsigned long long)s.acks_sent);
        printf("  ACKs received:         %llu\n", (unsigned long long)s.acks_received);
        printf("  NACKs sent:            %llu\n", (unsigned long long)s.nacks_sent);
        printf("  NACKs received:        %llu\n", (unsigned long long)s.nacks_received);
        printf("  Retransmissions:       %llu\n", (unsigned long long)s.retransmissions);
        printf("  Packets lost:          %llu\n", (unsigned long long)s.packets_lost);
        printf("  Duplicates dropped:    %llu\n", (unsigned long long)s.duplicates_dropped);
        printf("  CRC errors:            %llu\n", (unsigned long long)s.crc_errors);
        printf("  Delivery rate:         %.1f%%\n", s.delivery_rate());
        printf("============================\n\n");
    }

private:
    PacketManager manager_;
};

} // namespace rmavlink
