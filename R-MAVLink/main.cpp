// main.cpp  —  R-MAVLink demo entry point (UDP mode)
// ─────────────────────────────────────────────────────────────────────────────
//  Launches either as SENDER or RECEIVER depending on first CLI argument.
//
//  Usage:
//    Terminal 1 (receiver):  ./rmavlink recv
//    Terminal 2 (sender):    ./rmavlink send
//
//  Or for quick loopback test with simulated loss:
//    ./rmavlink loopback [loss_pct]
//
//  Build:
//    g++ -std=c++17 -O2 -I include \
//        core/reliable_protocol.cpp \
//        core/packet_manager.cpp \
//        transport/udp.cpp \
//        mavlink/mavlink_parser.cpp \
//        main.cpp -o rmavlink
// ─────────────────────────────────────────────────────────────────────────────

#include "include/reliable_protocol.h"
#include "include/packet_manager.h"
#include "transport/udp.cpp"      // Include transport implementations
#include "mavlink/mavlink_parser.cpp"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

using namespace rmavlink;
using namespace std::chrono;

static std::atomic<bool> g_running{true};

void handle_signal(int) { g_running = false; }

// ─────────────────────────────────────────────
//  Sender mode
// ─────────────────────────────────────────────
void run_sender() {
    printf("[SENDER] Starting — sending to 127.0.0.1:14551\n");

    UdpTransport transport("127.0.0.1", 14551, 14550);
    PacketManager manager(&transport);

    manager.on_packet_lost = [](uint16_t seq) {
        printf("[SENDER] !! Packet seq=%u permanently lost after %d retries\n",
               seq, MAX_RETRIES);
    };

    MavlinkParser mav_parser;

    // Build a fake HEARTBEAT payload (14 bytes of zeros)
    uint8_t heartbeat[14] = {};
    uint8_t mav_frame[280];
    size_t  mav_len = mav_parser.build_packet(mav_frame, sizeof(mav_frame),
                                               0 /* HEARTBEAT msgid */,
                                               heartbeat, sizeof(heartbeat));

    uint32_t sent = 0;
    auto     last_stats = steady_clock::now();

    printf("[SENDER] Sending MAVLink HEARTBEAT packets at ~10 Hz\n");
    printf("[SENDER] Press Ctrl+C to stop.\n\n");

    while (g_running) {
        // Send one HEARTBEAT per 100 ms (10 Hz)
        Error err = manager.send(mav_frame, mav_len);
        if (err == Error::OK) {
            sent++;
            if (sent % 10 == 0)
                printf("[SENDER] Sent packet #%u  (window used: %zu/%d)\n",
                       sent, manager.window_used(), WINDOW_SIZE);
        }

        // Drive retransmission engine
        for (int i = 0; i < 20; ++i) {
            manager.tick();
            std::this_thread::sleep_for(milliseconds(5));
        }

        // Print stats every 10 s
        auto now = steady_clock::now();
        if (duration_cast<seconds>(now - last_stats).count() >= 10) {
            const Stats& s = manager.stats();
            printf("\n[SENDER STATS] sent=%llu  retx=%llu  lost=%llu  delivery=%.1f%%\n\n",
                   (unsigned long long)s.packets_sent,
                   (unsigned long long)s.retransmissions,
                   (unsigned long long)s.packets_lost,
                   s.delivery_rate());
            last_stats = now;
        }
    }

    const Stats& s = manager.stats();
    printf("\n[SENDER] Final stats:\n");
    printf("  Packets sent:    %llu\n", (unsigned long long)s.packets_sent);
    printf("  Retransmissions: %llu\n", (unsigned long long)s.retransmissions);
    printf("  Packets lost:    %llu\n", (unsigned long long)s.packets_lost);
    printf("  Delivery rate:   %.1f%%\n", s.delivery_rate());
}

// ─────────────────────────────────────────────
//  Receiver mode
// ─────────────────────────────────────────────
void run_receiver() {
    printf("[RECEIVER] Starting — listening on :14551\n");

    UdpTransport transport("127.0.0.1", 14550, 14551);
    PacketManager manager(&transport);

    MavlinkParser mav_parser;
    uint32_t received = 0;

    manager.on_data_received = [&](const uint8_t* payload, size_t len, uint16_t seq) {
        received++;
        // Parse MAVLink from the reliable payload
        mav_parser.feed_bytes(payload, len,
            [&](const uint8_t* frame, size_t flen, uint32_t msgid) {
                printf("[RECEIVER] R-MAVLink seq=%-5u  MAVLink msgid=%-4u  "
                       "len=%-4zu  total_rx=%u\n",
                       seq, msgid, flen, received);
            });
    };

    printf("[RECEIVER] Waiting for packets. Press Ctrl+C to stop.\n\n");

    while (g_running) {
        manager.tick();
        std::this_thread::sleep_for(milliseconds(1));
    }

    const Stats& s = manager.stats();
    printf("\n[RECEIVER] Final stats:\n");
    printf("  Packets received:   %llu\n", (unsigned long long)s.packets_received);
    printf("  ACKs sent:          %llu\n", (unsigned long long)s.acks_sent);
    printf("  Duplicates dropped: %llu\n", (unsigned long long)s.duplicates_dropped);
    printf("  CRC errors:         %llu\n", (unsigned long long)s.crc_errors);
}

// ─────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    printf("╔══════════════════════════════════════╗\n");
    printf("║   R-MAVLink Reliable Protocol Demo   ║\n");
    printf("╚══════════════════════════════════════╝\n\n");

    if (argc < 2 || strcmp(argv[1], "send") == 0) {
        run_sender();
    } else if (strcmp(argv[1], "recv") == 0) {
        run_receiver();
    } else if (strcmp(argv[1], "loopback") == 0) {
        // Run simulation test inline
        printf("Running built-in loopback simulation...\n");
        printf("(For full stress test, build and run tests/packet_loss_sim.cpp)\n\n");
        run_sender();
    } else {
        printf("Usage: %s [send|recv|loopback]\n", argv[0]);
        return 1;
    }

    return 0;
}
