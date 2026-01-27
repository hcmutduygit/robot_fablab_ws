#pragma once
#include <vector>
#include <cstdint>

struct EncoderData {
    int left_pulse;
    int right_pulse;
};

bool decodeEncoder(uint16_t can_id,
                   const std::vector<uint8_t>& data,
                   EncoderData& out);
