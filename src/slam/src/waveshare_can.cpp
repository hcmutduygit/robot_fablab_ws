#include "waveshare_can.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <vector>
#include <thread>
#include <utility>

WaveshareCAN::WaveshareCAN(const std::string& port,
                           uint32_t baudrate,
                           float timeout)
    : port_(port),
      baudrate_(baudrate),
      timeout_(timeout),
      fd_(-1),
      rx_running_(false)
{
}

WaveshareCAN::~WaveshareCAN() {
    close();
}

void WaveshareCAN::open() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == -1) {
        throw std::runtime_error(
            "Failed to open serial port: " + port_ + " (" + std::strerror(errno) + ")");
    }

    struct termios options{};
    tcgetattr(fd_, &options);
    cfsetispeed(&options, B2000000);
    cfsetospeed(&options, B2000000);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;+
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VTIME] = static_cast<int>(timeout_ * 10);
    options.c_cc[VMIN]  = 0;

    tcsetattr(fd_, TCSANOW, &options);
    tcflush(fd_, TCIFLUSH);
}

void WaveshareCAN::close() {
    rx_running_ = false;
    if (rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->join();
    }
    if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
            std::cout << "🔌 Serial port closed\n";
        }
}

void WaveshareCAN::send(uint16_t can_id,
                        const std::vector<uint8_t>& data)
{
    if (fd_ == -1)
        throw std::runtime_error("Serial port not open");

    std::vector<uint8_t> frame;
    frame.push_back(0xAA);
    frame.push_back(0xC8);
    frame.push_back(can_id & 0xFF);
    frame.push_back((can_id >> 8) & 0xFF);

    std::vector<uint8_t> payload = data;
    payload.resize(8, 0);
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(0x55);

    write(fd_, frame.data(), frame.size());
}

std::pair<uint16_t, std::vector<uint8_t>> WaveshareCAN::receive()
{
    if (fd_ == -1) {
        throw std::runtime_error("Serial port is not open. Call open() first.");
    }

    uint8_t b;
    // Wait for start byte (0xAA)
    while (true) {
        if (read_exact(&b, 1) && b == 0xAA) {
            break;
        }
    }

    // Read header (3 bytes)
    std::vector<uint8_t> header(3);
    if (!read_exact(header.data(), 3)) {
        throw std::runtime_error("Failed to read header");
    }

    // Read CMD byte
    uint8_t cmd;
    if (!read_exact(&cmd, 1)) {
        throw std::runtime_error("Failed to read CMD byte");
    }
        
    uint8_t idl;
    if (!read_exact(&idl, 1)) {
            throw std::runtime_error("Failed to read IDL");
    }

    // Read IDH (3 bytes)
    std::vector<uint8_t> idh(3);
    if (!read_exact(idh.data(), 3)) {
        throw std::runtime_error("Failed to read IDH");
    }

    // Read length (1 byte)
    uint8_t length;
    if (!read_exact(&length, 1)) {
        throw std::runtime_error("Failed to read length");
    }

    // Read 8 data bytes
    std::vector<uint8_t> data(8);
    if (!read_exact(data.data(), 8)) {
        throw std::runtime_error("Failed to read data bytes");
    }

    // Read tail byte
    uint8_t tail;
    if (!read_exact(&tail, 1)) {
            throw std::runtime_error("Failed to read tail byte");
    }
    
    uint16_t can_id = idl | (idh[0] << 8);
    return {can_id, data};
}

void WaveshareCAN::start_receive_loop(Callback callback) {
    if (rx_thread_ && rx_thread_->joinable()) {
        std::cout << "🔄 Receive loop already running.\n";
        return;
    }

    rx_running_ = true; 
    rx_thread_ = std::make_unique<std::thread>(
        &WaveshareCAN::receive_worker, this, callback
    );
    std::cout << "🔄 Receive loop started (thread)\n";
}

bool WaveshareCAN::read_exact(uint8_t* buffer, size_t len) {
    size_t bytes_read = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (bytes_read < len) {
        ssize_t n = read(fd_, buffer + bytes_read, len - bytes_read);
        if (n > 0) {
            bytes_read += n;
        } else if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            // Timeout or no data available
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= timeout_ * 1000) {
                return false; // Timeout
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            std::cerr << "Read error: " << std::strerror(errno) << "\n";
            return false;
        }
    }
    return true;
}

void WaveshareCAN::receive_worker(Callback callback) {
    while (rx_running_) {
        try {
            auto result = receive();
            auto can_id = result.first;
            auto data   = result.second;
            callback(can_id, data);
        } catch (const std::exception& e) {
            std::cerr << "Error in receive loop: " << e.what() << ". Retrying...\n";
            // Clear input buffer to resynchronize
            tcflush(fd_, TCIFLUSH);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

