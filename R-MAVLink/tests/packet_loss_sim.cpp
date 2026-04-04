// packet_loss_sim.cpp
// ─────────────────────────────────────────────────────────────────────────────
//  R-MAVLink  —  Packet Loss Simulation & Stress Test
//
//  Simulates a noisy channel between two in-process endpoints.
//  Injects configurable:
//    • packet loss probability
//    • artificial latency
//    • duplicate packets
//
//  Run:
//    g++ -std=c++17 -I../include tests/packet_loss_sim.cpp -o sim && ./sim
// ─────────────────────────────────────────────────────────────────────────────

#include "../include/packet_manager.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cassert>
#include <vector>
#include <deque>
#include <thread>
#include <chrono>
#include <random>
#include <atomic>
#include <functional>

using namespace rmavlink;
using namespace std::chrono;

// ─────────────────────────────────────────────
//  SimChannel  — bidirectional lossy pipe
// ─────────────────────────────────────────────
struct ChannelConfig {
    float    loss_rate  = 0.0f;   // 0.0–1.0
    float    dup_rate   = 0.0f;   // Duplicate packet probability
    uint32_t min_delay  = 0;      // ms
    uint32_t max_delay  = 0;      // ms
};

struct DelayedPacket {
    std::vector<uint8_t> data;
    uint64_t             deliver_at_ms;
};

class SimChannel {
public:
    explicit SimChannel(ChannelConfig cfg)
        : cfg_(cfg), rng_(std::random_device{}()) {}

    /// Push a packet from A→B side.
    void push(const uint8_t* data, size_t len) {
        if (drop()) return;  // Simulate loss

        uint64_t delay = random_delay();
        uint64_t now   = now_ms();

        DelayedPacket pkt;
        pkt.data.assign(data, data + len);
        pkt.deliver_at_ms = now + delay;
        queue_.push_back(std::move(pkt));

        if (dup()) {
            // Duplicate: push same packet again with a tiny extra delay
            DelayedPacket dup_pkt;
            dup_pkt.data.assign(data, data + len);
            dup_pkt.deliver_at_ms = now + delay + 2;
            queue_.push_back(std::move(dup_pkt));
        }
    }

    /// Drain packets whose delay has elapsed. Calls cb(data, len) for each.
    void drain(std::function<void(const uint8_t*, size_t)> cb) {
        uint64_t now = now_ms();
        while (!queue_.empty() && queue_.front().deliver_at_ms <= now) {
            cb(queue_.front().data.data(), queue_.front().data.size());
            queue_.pop_front();
        }
    }

    size_t pending() const { return queue_.size(); }

private:
    ChannelConfig               cfg_;
    std::mt19937                rng_;
    std::deque<DelayedPacket>   queue_;

    bool drop() {
        if (cfg_.loss_rate <= 0.0f) return false;
        std::uniform_real_distribution<float> d(0.0f, 1.0f);
        return d(rng_) < cfg_.loss_rate;
    }

    bool dup() {
        if (cfg_.dup_rate <= 0.0f) return false;
        std::uniform_real_distribution<float> d(0.0f, 1.0f);
        return d(rng_) < cfg_.dup_rate;
    }

    uint64_t random_delay() {
        if (cfg_.min_delay == cfg_.max_delay) return cfg_.min_delay;
        std::uniform_int_distribution<uint32_t> d(cfg_.min_delay, cfg_.max_delay);
        return d(rng_);
    }

    static uint64_t now_ms() {
        return (uint64_t)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }
};

// ─────────────────────────────────────────────
//  SimTransport  — wraps SimChannel for both ends
// ─────────────────────────────────────────────
class SimTransport : public ITransport {
public:
    SimTransport(SimChannel* send_channel, SimChannel* recv_channel)
        : send_ch_(send_channel), recv_ch_(recv_channel) {}

    int send(const uint8_t* data, size_t len) override {
        send_ch_->push(data, len);
        return (int)len;
    }

    int recv(uint8_t* buf, size_t max_len) override {
        // If we have a buffered packet from a previous drain, return it first
        if (!rx_buf_.empty()) {
            size_t len = rx_buf_.front().size();
            if (len <= max_len) {
                std::memcpy(buf, rx_buf_.front().data(), len);
                rx_buf_.pop_front();
                return (int)len;
            }
            rx_buf_.pop_front(); // discard oversized
        }

        // Drain newly-arrived packets into rx_buf_
        recv_ch_->drain([&](const uint8_t* data, size_t len) {
            rx_buf_.push_back(std::vector<uint8_t>(data, data + len));
        });

        if (!rx_buf_.empty()) {
            size_t len = rx_buf_.front().size();
            if (len <= max_len) {
                std::memcpy(buf, rx_buf_.front().data(), len);
                rx_buf_.pop_front();
                return (int)len;
            }
            rx_buf_.pop_front();
        }
        return 0;
    }

    uint64_t now_ms() override {
        return (uint64_t)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }

    bool is_connected() override { return true; }

private:
    SimChannel*                         send_ch_;
    SimChannel*                         recv_ch_;
    std::deque<std::vector<uint8_t>>    rx_buf_;
};

// ─────────────────────────────────────────────
//  Test runner
// ─────────────────────────────────────────────
struct TestResult {
    int     packets_queued   = 0;
    int     packets_received = 0;
    float   loss_rate_pct    = 0.0f;
    float   delivery_rate    = 0.0f;
    int64_t duration_ms      = 0;
    uint64_t retransmissions = 0;
};

TestResult run_test(const char* name, ChannelConfig channel_cfg,
                    int num_packets = 200, int run_ms = 5000)
{
    printf("\n─────────────────────────────────────────────\n");
    printf(" TEST: %s\n", name);
    printf("  loss=%.0f%%  dup=%.0f%%  delay=%u–%ums  packets=%d\n",
           channel_cfg.loss_rate * 100, channel_cfg.dup_rate * 100,
           channel_cfg.min_delay, channel_cfg.max_delay, num_packets);
    printf("─────────────────────────────────────────────\n");

    // A→B channel (DATA), B→A channel (ACK)
    SimChannel a2b(channel_cfg);
    SimChannel b2a({ 0.0f, 0.0f, 1, 2 });  // ACK channel: near-perfect

    SimTransport transport_a(&a2b, &b2a);  // Sender
    SimTransport transport_b(&b2a, &a2b);  // Receiver

    PacketManager sender(&transport_a);
    PacketManager receiver(&transport_b);

    std::atomic<int> received{0};

    receiver.on_data_received = [&](const uint8_t* p, size_t len, uint16_t seq) {
        (void)p; (void)len;
        received++;
    };

    // Build a fake MAVLink-like payload (32 bytes of incrementing data)
    uint8_t payload[32];
    for (int i = 0; i < 32; ++i) payload[i] = (uint8_t)i;

    auto start = steady_clock::now();

    int queued = 0;
    bool sending_done = false;
    int64_t elapsed = 0;

    while (elapsed < run_ms) {
        elapsed = duration_cast<milliseconds>(steady_clock::now() - start).count();

        // Queue packets up-front (non-blocking via window)
        if (!sending_done && queued < num_packets) {
            if (!sender.window_full()) {
                payload[0] = (uint8_t)queued;  // Tag each packet
                sender.send(payload, sizeof(payload));
                queued++;
            }
        } else if (queued >= num_packets) {
            sending_done = true;
        }

        sender.tick();
        receiver.tick();

        std::this_thread::sleep_for(microseconds(500));
    }

    const Stats& s = sender.stats();

    TestResult r;
    r.packets_queued   = queued;
    r.packets_received = received.load();
    r.delivery_rate    = (queued > 0) ? 100.0f * r.packets_received / queued : 0.0f;
    r.loss_rate_pct    = channel_cfg.loss_rate * 100.0f;
    r.duration_ms      = elapsed;
    r.retransmissions  = s.retransmissions;

    printf("  Queued:          %d\n", r.packets_queued);
    printf("  Received:        %d\n", r.packets_received);
    printf("  Delivery rate:   %.1f%%\n", r.delivery_rate);
    printf("  Retransmissions: %llu\n", (unsigned long long)r.retransmissions);
    printf("  Duplicates drop: %llu\n", (unsigned long long)s.duplicates_dropped);
    printf("  CRC errors:      %llu\n", (unsigned long long)s.crc_errors);
    printf("  Duration:        %lld ms\n", (long long)r.duration_ms);

    return r;
}

// ─────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────
int main() {
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║     R-MAVLink  Packet Loss Simulation     ║\n");
    printf("╚═══════════════════════════════════════════╝\n");

    std::vector<TestResult> results;

    // ── Phase 1: Ideal link ────────────────────
    results.push_back(run_test("Ideal link (0% loss)",
        { 0.0f, 0.0f, 1, 5 }, 100, 3000));

    // ── Phase 2: Moderate loss ─────────────────
    results.push_back(run_test("Moderate loss (20%)",
        { 0.20f, 0.0f, 5, 20 }, 100, 6000));

    // ── Phase 3: Heavy loss ────────────────────
    results.push_back(run_test("Heavy loss (40%)",
        { 0.40f, 0.0f, 5, 30 }, 100, 10000));

    // ── Phase 4: Loss + duplicates ─────────────
    results.push_back(run_test("Loss + duplicates (30%+10%)",
        { 0.30f, 0.10f, 5, 25 }, 100, 8000));

    // ── Phase 5: High latency ──────────────────
    results.push_back(run_test("High latency (50–120ms, 10% loss)",
        { 0.10f, 0.0f, 50, 120 }, 50, 15000));

    // ── Summary table ──────────────────────────
    printf("\n╔══════════════════════════════════════════════════════════════╗\n");
    printf("║                     RESULTS SUMMARY                         ║\n");
    printf("╠══════════════╦══════════╦══════════╦═══════════╦════════════╣\n");
    printf("║ Loss Simul'd ║ Queued   ║ Received ║ Delivery  ║ Retransmit ║\n");
    printf("╠══════════════╬══════════╬══════════╬═══════════╬════════════╣\n");
    for (const auto& r : results) {
        printf("║ %8.0f%%    ║ %8d ║ %8d ║ %8.1f%% ║ %10llu ║\n",
               r.loss_rate_pct, r.packets_queued, r.packets_received,
               r.delivery_rate, (unsigned long long)r.retransmissions);
    }
    printf("╚══════════════╩══════════╩══════════╩═══════════╩════════════╝\n");

    printf("\n[BASELINE] Normal MAVLink delivery on 30%% loss: ~70%%\n");
    printf("[R-MAVLink] Expected delivery on 30%% loss:  ~98%%\n\n");

    return 0;
}
