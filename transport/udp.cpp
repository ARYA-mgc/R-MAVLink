// udp.cpp  —  UDP transport for R-MAVLink (Linux / macOS / Windows)
// Implements the ITransport interface over a UDP socket.
// Use this for PC↔PC simulation and Raspberry Pi ↔ GCS links.

#include "../include/reliable_protocol.h"

#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <chrono>

// ── Platform socket headers ─────────────────────────────────────────────────
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  typedef int socklen_t;
  #define SOCK_NONBLOCK 0
  #define close closesocket
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <errno.h>
#endif

namespace rmavlink {

// ─────────────────────────────────────────────
//  UdpTransport
//
//  Binds to `local_port` (receive),
//  sends to `remote_host:remote_port`.
// ─────────────────────────────────────────────
class UdpTransport : public ITransport {
public:
    UdpTransport(const char* remote_host, uint16_t remote_port, uint16_t local_port) {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
#endif
        sockfd_ = (int)socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0)
            throw std::runtime_error("UDP: socket() failed");

        // Bind local port (receive)
        sockaddr_in local{};
        local.sin_family      = AF_INET;
        local.sin_addr.s_addr = INADDR_ANY;
        local.sin_port        = htons(local_port);
        if (bind(sockfd_, (sockaddr*)&local, sizeof(local)) < 0) {
            close(sockfd_);
            throw std::runtime_error("UDP: bind() failed");
        }

        // Build remote address (send)
        std::memset(&remote_, 0, sizeof(remote_));
        remote_.sin_family = AF_INET;
        remote_.sin_port   = htons(remote_port);
#ifdef _WIN32
        inet_pton(AF_INET, remote_host, &remote_.sin_addr);
#else
        inet_aton(remote_host, &remote_.sin_addr);
#endif

        // Non-blocking receive
        set_nonblocking(sockfd_);

        printf("[UDP] Listening on :%u  →  %s:%u\n", local_port, remote_host, remote_port);
    }

    ~UdpTransport() override {
        close(sockfd_);
#ifdef _WIN32
        WSACleanup();
#endif
    }

    int send(const uint8_t* data, size_t len) override {
        ssize_t n = sendto(sockfd_, (const char*)data, (int)len, 0,
                           (sockaddr*)&remote_, sizeof(remote_));
        return (int)n;
    }

    int recv(uint8_t* buf, size_t max_len) override {
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(sockfd_, (char*)buf, (int)max_len, 0,
                             (sockaddr*)&from, &from_len);
        if (n < 0) {
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) return 0;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
#endif
            return -1;
        }
        return (int)n;
    }

    uint64_t now_ms() override {
        using namespace std::chrono;
        return (uint64_t)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }

    bool is_connected() override { return sockfd_ >= 0; }

private:
    int            sockfd_;
    sockaddr_in    remote_;

    static void set_nonblocking(int fd) {
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(fd, FIONBIO, &mode);
#else
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
    }
};

} // namespace rmavlink
