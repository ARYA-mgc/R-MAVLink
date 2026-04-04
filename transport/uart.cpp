// uart.cpp  —  UART / Serial transport for R-MAVLink
// Implements ITransport over a POSIX serial port (Linux).
// Use for Raspberry Pi ↔ STM32 / Pixhawk links.

#include "../include/reliable_protocol.h"

#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <chrono>

#ifndef _WIN32
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <errno.h>
#endif

namespace rmavlink {

// ─────────────────────────────────────────────
//  UartTransport
//
//  Opens a TTY device at the given baud rate.
//  Common devices:
//    /dev/ttyUSB0   (USB-UART adapter)
//    /dev/ttyAMA0   (RPi hardware UART)
//    /dev/ttyS0     (PC COM1)
// ─────────────────────────────────────────────
class UartTransport : public ITransport {
public:
    UartTransport(const char* device, int baud = 115200) {
#ifdef _WIN32
        throw std::runtime_error("UART: Windows not supported in this build");
#else
        fd_ = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
            throw std::runtime_error(std::string("UART: cannot open ") + device);

        configure(baud);
        printf("[UART] Opened %s @ %d baud\n", device, baud);
#endif
    }

#ifndef _WIN32
    ~UartTransport() override {
        if (fd_ >= 0) close(fd_);
    }

    int send(const uint8_t* data, size_t len) override {
        ssize_t n = write(fd_, data, len);
        return (int)n;
    }

    int recv(uint8_t* buf, size_t max_len) override {
        ssize_t n = read(fd_, buf, max_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        return (int)n;
    }

    uint64_t now_ms() override {
        using namespace std::chrono;
        return (uint64_t)duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count();
    }

    bool is_connected() override { return fd_ >= 0; }

private:
    int fd_ = -1;

    void configure(int baud) {
        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0)
            throw std::runtime_error("UART: tcgetattr() failed");

        // Baud rate
        speed_t speed = baud_to_speed(baud);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8N1, no flow control
        tty.c_cflag &= ~PARENB;    // No parity
        tty.c_cflag &= ~CSTOPB;    // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |=  CS8;       // 8 data bits
        tty.c_cflag &= ~CRTSCTS;   // No hardware flow control
        tty.c_cflag |=  CREAD | CLOCAL;

        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        // Non-blocking read
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
            throw std::runtime_error("UART: tcsetattr() failed");

        tcflush(fd_, TCIOFLUSH);
    }

    static speed_t baud_to_speed(int baud) {
        switch (baud) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:     return B115200;
        }
    }
#endif
};

} // namespace rmavlink
