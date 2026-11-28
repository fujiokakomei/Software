#include "upperVL53.hpp"

#include <limits.h>

#include "Debug.hpp"

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

bool UpperVL53s::receiveDistance(uint16_t out[NUM_SENSORS], ReceiveDebugInfo* debugInfo)
{
    while (_rp2040.available())
    {
        const uint8_t byte = static_cast<uint8_t>(_rp2040.read());
        if (debugInfo != nullptr)
        {
            ++debugInfo->bytesRead;
            if (debugInfo->rawDataSize < DEBUG_RAW_DATA_SIZE)
            {
                debugInfo->rawData[debugInfo->rawDataSize++] = byte;
            }
            else
            {
                debugInfo->rawDataTruncated = true;
            }
        }

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
                if (debugInfo != nullptr)
                {
                    ++debugInfo->validFrames;
                }
                for (size_t i = 0; i < NUM_SENSORS; ++i)
                {
                    out[i] = static_cast<uint16_t>(_payload[i * 2]) |
                             (static_cast<uint16_t>(_payload[i * 2 + 1]) << 8);
                }
                return true;
            }
            if (debugInfo != nullptr)
            {
                ++debugInfo->checksumErrors;
            }
            break;
        }
    }
    return false;
}

void UpperVL53s::begin(uint32_t baud)
{
    _rp2040.begin(baud);
    _rp2040.addMemoryForRead(_extraRxBuffer, sizeof(_extraRxBuffer));
}

unsigned long UpperVL53s::lastFrameAgeMs() const
{
    return _hasFrame ? millis() - _lastFrameMs : ULONG_MAX;
}

void UpperVL53s::getVL53s(uint16_t _distance_data[NUM_SENSORS], ReceiveDebugInfo* debugInfo)
{
    uint16_t received[NUM_SENSORS];
    if (debugInfo != nullptr)
    {
        *debugInfo = {};
    }

    if (static_cast<size_t>(_rp2040.available()) >= STALE_BUFFER_BYTES)
    {
        // 長時間読まなかったため受信バッファが溢れ、古いデータしか残っていない。
        // 捨てて、次に届く新しいフレームを待つ。
        DBG("VL53 RX buffer overflow (%d bytes), drop old data", _rp2040.available());
        while (_rp2040.available())
        {
            _rp2040.read();
        }
        _receiveState = ReceiveState::WaitForV;

        const unsigned long waitStart = millis();
        while (millis() - waitStart < FRESH_FRAME_TIMEOUT_MS)
        {
            if (receiveDistance(received, debugInfo))
            {
                memcpy(distance_data, received, PAYLOAD_SIZE);
                _lastFrameMs = millis();
                _hasFrame = true;
                break;
            }
        }
    }

    // 到着済みの完全なフレームをすべて処理し、最後に検証できた値を保持する。
    while (receiveDistance(received, debugInfo))
    {
        memcpy(distance_data, received, PAYLOAD_SIZE);
        _lastFrameMs = millis();
        _hasFrame = true;
    }

    memcpy(_distance_data, distance_data, PAYLOAD_SIZE);

    // RP2040からのフレームが途絶えていたら警告（1秒に1回まで）
    if (!_hasFrame || millis() - _lastFrameMs > 500)
    {
        DBG_EVERY(1000, "WARN VL53 no frame from RP2040 for %ld ms",
                  _hasFrame ? static_cast<long>(millis() - _lastFrameMs) : -1L);
    }
}
}
