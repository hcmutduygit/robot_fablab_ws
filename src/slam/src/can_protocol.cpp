#include "can_protocol.h"
#include <cstring>

bool decodeEncoder(uint16_t can_id,
                   const std::vector<uint8_t>& data,
                   EncoderData& out)
{
    if (can_id != 0x11 || data.size() != 8)
        return false;

    std::memcpy(&out.left_pulse,  &data[0], 4);
    std::memcpy(&out.right_pulse, &data[4], 4);
    return true;
}
