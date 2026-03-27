#ifndef WAVESHARE_CAN_H
#define WAVESHARE_CAN_H

#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <thread>
#include <memory>
#include <cstdint>

class WaveshareCAN {
public:
    using Callback = std::function<void(uint16_t, const std::vector<uint8_t>&)>;

    WaveshareCAN(const std::string& port = "/dev/ttyUSB0",
                 uint32_t baudrate = 2000000,
                 float timeout = 1.0);

    ~WaveshareCAN();

    void open();
    void close();
    void send(uint16_t can_id, const std::vector<uint8_t>& data);
    void start_receive_loop(Callback callback);

private:
    bool read_exact(uint8_t* buffer, size_t len);
    std::pair<uint16_t, std::vector<uint8_t>> receive();
    void receive_worker(Callback callback);

private:
    std::string port_;
    uint32_t baudrate_;
    float timeout_;
    int fd_;

    std::atomic<bool> rx_running_;
    std::unique_ptr<std::thread> rx_thread_;
};

#endif
