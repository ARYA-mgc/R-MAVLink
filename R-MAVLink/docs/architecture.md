# R-MAVLink Architecture

> **R-MAVLink** — Reliable MAVLink Protocol  
> Version 1.0.0 | Research-Grade Implementation

---

## Overview

R-MAVLink wraps standard MAVLink packets inside a lightweight reliable transport layer, providing:

- Guaranteed packet delivery via ARQ (Automatic Repeat reQuest)
- Sliding-window throughput optimisation
- NACK-accelerated retransmission
- Duplicate detection & rejection
- Pluggable transport (UDP · UART · custom)

```
┌──────────────────────────────────────────────────────────┐
│                     Application Layer                     │
│           (GCS / Flight Controller / Sensor Node)        │
└───────────────────────────┬──────────────────────────────┘
                            │ MAVLink messages
┌───────────────────────────▼──────────────────────────────┐
│                   MavlinkParser                           │
│          Build & parse raw MAVLink v2 frames             │
└───────────────────────────┬──────────────────────────────┘
                            │ raw MAVLink bytes (≤263 B)
┌───────────────────────────▼──────────────────────────────┐
│               ReliableProtocol  (API layer)               │
│         send() / tick() / feed() / on_receive            │
└───────────────────────────┬──────────────────────────────┘
                            │
┌───────────────────────────▼──────────────────────────────┐
│                    PacketManager                          │
│  • Builds ReliableFrame  (header + payload + CRC)        │
│  • Manages sliding window  [base … head)                 │
│  • Retransmits on timeout / NACK                         │
│  • Detects & drops duplicates on receive                 │
└───────────────────────────┬──────────────────────────────┘
                            │ raw bytes
┌───────────────────────────▼──────────────────────────────┐
│                    ITransport                             │
│          UdpTransport  │  UartTransport  │  custom       │
└──────────────────────────────────────────────────────────┘
```

---

## Packet Format

### ReliableFrame wire layout

```
 0       1       2       3       4       5       6       7
 ┌───────┬───────┬───────┬───────┬───────┬───────┬───────┐
 │   magic (0xAF42)  │     seq_id        │msg_typ│       │
 ├───────┴───────┴───────┴───────┴───────┤       ├───────┤
 │     timestamp (ms)    │ payload_len   │ hdr_crc (LE)  │
 ├───────────────────────────────────────┴───────────────┤
 │                  MAVLink payload (0–280 B)             │
 ├───────────────────────────────────────────────────────┤
 │                  frame_crc (CRC-16, LE)               │
 └───────────────────────────────────────────────────────┘
```

| Field         | Bytes | Description                                      |
|---------------|-------|--------------------------------------------------|
| `magic`       | 2     | `0xAF42` — frame sync word                       |
| `seq_id`      | 2     | Monotonic sequence number, wraps at 65535        |
| `msg_type`    | 1     | `DATA=1  ACK=2  NACK=3  PING=4  PONG=5`         |
| `timestamp`   | 2     | Sender clock mod 65536 ms                        |
| `payload_len` | 1     | MAVLink payload bytes (0 for ACK/NACK)           |
| `header_crc`  | 2     | CRC-16 of all header fields except itself        |
| payload       | 0–280 | Raw MAVLink frame                                |
| `frame_crc`   | 2     | CRC-16 of header + payload                       |

**Total overhead vs bare MAVLink: +10 bytes per packet**

---

## Protocol Flow

### Normal DATA / ACK flow

```
Sender                              Receiver
  │                                     │
  │──── DATA(seq=101, ts=500) ─────────►│
  │                                     │ validate CRC
  │                                     │ check seq > last_recv
  │◄─── ACK(seq=101) ──────────────────│
  │                                     │
  │──── DATA(seq=102) ─────────────────►│
  │◄─── ACK(seq=102) ──────────────────│
```

### Retransmission on timeout

```
Sender                              Receiver
  │                                     │
  │──── DATA(seq=103) ─── LOST ─── ✗   │  (packet dropped)
  │                                     │
  │  [timeout = 100 ms]                 │
  │                                     │
  │──── DATA(seq=103) [retry 1] ───────►│
  │◄─── ACK(seq=103) ──────────────────│
```

### NACK fast-recovery

```
Sender                              Receiver
  │                                     │
  │──── DATA(seq=104) ─── CRC error ─► │
  │◄─── NACK(seq=104) ─────────────────│  (immediate)
  │──── DATA(seq=104) [immediate] ─────►│
  │◄─── ACK(seq=104) ──────────────────│
```

### Sliding Window (WINDOW_SIZE = 8)

```
  Sender window:  [101, 102, 103, 104, 105, 106, 107, 108]
                    ↑                                   ↑
                  base                               head

  After ACK(101):
  Window slides:  [102, 103, 104, 105, 106, 107, 108, 109]
```

---

## Key Design Decisions

### 1. Header-only `PacketManager`
`packet_manager.h` is header-only so it can be used on embedded targets (STM32, bare-metal) without a linker step — just `#include` and go.

### 2. Sequence wrap-around
`seq_id` is `uint16_t` (0–65535). Comparison uses half-range arithmetic to correctly handle wrap:
```cpp
bool seq_le(uint16_t a, uint16_t b) {
    return ((b - a) & 0x8000) == 0 && a != b + 1;
}
```

### 3. Two CRCs
- `header_crc` — fast validation of the header alone; avoids allocating a receive buffer for a frame with a corrupt length field.
- `frame_crc` — validates the entire frame (header + payload).

### 4. Transport abstraction
All I/O flows through `ITransport`. Swapping UART ↔ UDP ↔ simulation requires zero changes to `PacketManager`.

---

## Configuration Parameters

| Parameter      | Default | Range     | Effect                             |
|----------------|---------|-----------|------------------------------------|
| `TIMEOUT_MS`   | 100 ms  | 50–500 ms | ACK wait before retransmit         |
| `MAX_RETRIES`  | 4       | 1–10      | Retransmits before marking lost    |
| `WINDOW_SIZE`  | 8       | 1–32      | In-flight packets simultaneously   |

**Tuning guide:**
- Increase `TIMEOUT_MS` for high-latency links (LTE, long-range RF)
- Decrease `WINDOW_SIZE` on memory-constrained targets
- Increase `MAX_RETRIES` for ultra-reliable mission-critical data

---

## Performance Model

Given channel loss probability `p` and `MAX_RETRIES = n`:

```
P(packet permanently lost) = p^(n+1)
```

| Channel Loss | MAX_RETRIES=3 | MAX_RETRIES=4 | MAX_RETRIES=5 |
|--------------|---------------|---------------|---------------|
| 10%          | 0.01%         | 0.001%        | 0.0001%       |
| 30%          | 0.81%         | 0.24%         | 0.073%        |
| 50%          | 6.25%         | 3.13%         | 1.56%         |

Compared to bare MAVLink (no retransmission):

| Channel Loss | MAVLink Delivery | R-MAVLink Delivery |
|--------------|------------------|--------------------|
| 10%          | ~90%             | ~99.99%            |
| 30%          | ~70%             | ~99.76%            |
| 50%          | ~50%             | ~96.9%             |

---

## Integration with MAVLink (Option A — Recommended)

```
[ReliableFrame header (10 B)] + [MAVLink v2 packet (10–280 B)] + [frame_crc (2 B)]
```

The MAVLink packet is treated as opaque binary data by R-MAVLink. You can use any MAVLink message as payload — HEARTBEAT, ATTITUDE, GPS, custom, etc.

```cpp
// Sender
uint8_t mav_buf[280];
size_t  mav_len = mav_parser.build_packet(mav_buf, sizeof(mav_buf), MSGID_HEARTBEAT, payload, len);
proto.send(mav_buf, mav_len);

// Receiver callback
proto.on_receive = [](const uint8_t* data, size_t len, uint16_t seq) {
    mav_parser.feed_bytes(data, len, [](const uint8_t* frame, size_t flen, uint32_t msgid) {
        // process MAVLink frame
    });
};
```

---

## File Structure

```
R-MAVLink/
├── include/
│   ├── reliable_protocol.h   ← Core types, ITransport, Stats
│   ├── packet_manager.h      ← Full sliding-window ARQ implementation
│   └── crc_utils.h           ← CRC-16/CCITT utilities
│
├── core/
│   ├── reliable_protocol.cpp ← ReliableProtocol public API
│   └── packet_manager.cpp    ← Factory helpers
│
├── transport/
│   ├── udp.cpp               ← UDP socket transport (Linux/macOS/Win)
│   └── uart.cpp              ← POSIX serial transport (Linux/RPi)
│
├── mavlink/
│   └── mavlink_parser.cpp    ← MAVLink v2 frame builder & parser
│
├── tests/
│   └── packet_loss_sim.cpp   ← In-process stress test
│
├── docs/
│   ├── architecture.md       ← This file
│   └── results.md            ← Benchmark results
│
├── main.cpp                  ← Demo: send / recv / loopback modes
├── CMakeLists.txt
├── Makefile
└── README.md
```
