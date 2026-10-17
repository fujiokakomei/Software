#include "upperVL53.hpp"

namespace uppervl53s
{
uint8_t UpperVL53s::crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

bool UpperVL53s::receiveDistance(uint16_t out[NUM_SENSORS])
{
    while (_rp2040.available())
    {
        const uint8_t byte = static_cast<uint8_t>(_rp2040.read());

        switch (_receiveState)
        {
        case ReceiveState::WaitForV:
            if (byte == kHeaderFirst)
            {
                _receiveState = ReceiveState::WaitForL;
            }
            break;

        case ReceiveState::WaitForL:
            if (byte == kHeaderSecond)
            {
                _payloadIndex = 0;
                _receiveState = ReceiveState::ReceivePayload;
            }
            else if (byte != kHeaderFirst)
            {
                _receiveState = ReceiveState::WaitForV;
            }
            break;

        case ReceiveState::ReceivePayload:
            _payload[_payloadIndex++] = byte;
            if (_payloadIndex == PAYLOAD_SIZE)
            {
                _receiveState = ReceiveState::ReceiveChecksum;
            }
            break;

        case ReceiveState::ReceiveChecksum:
            _receiveState = ReceiveState::WaitForV;
            if (crc8(_payload, PAYLOAD_SIZE) == byte)
            {
                for (size_t i = 0; i < NUM_SENSORS; ++i)
                {
                    out[i] = static_cast<uint16_t>(_payload[i * 2]) |
                             (static_cast<uint16_t>(_payload[i * 2 + 1]) << 8);
                }
                return true;
            }
            break;
        }
    }
    return false;
}

void UpperVL53s::getVL53s(uint16_t _distance_data[NUM_SENSORS])
{
    uint16_t received[NUM_SENSORS];

    // 到着済みの完全なフレームをすべて処理し、最後に検証できた値を保持する。
    while (receiveDistance(received))
    {
        memcpy(distance_data, received, PAYLOAD_SIZE);
    }

    memcpy(_distance_data, distance_data, PAYLOAD_SIZE);
}
}
