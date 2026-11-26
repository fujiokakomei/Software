#pragma once

#include <Arduino.h>

namespace uppervl53s
{
class UpperVL53s
{
public:
    static constexpr size_t NUM_SENSORS = 12;
    static constexpr size_t PAYLOAD_SIZE = NUM_SENSORS * sizeof(uint16_t);
    static constexpr size_t DEBUG_RAW_DATA_SIZE = 512;

    // getVL53s() 1 回の呼び出し中に Serial8 から読み出した情報。
    // テスト時に UART の通信状況と未加工の受信データを確認するために使う。
    struct ReceiveDebugInfo
    {
        size_t bytesRead = 0;
        size_t validFrames = 0;
        size_t checksumErrors = 0;
        size_t rawDataSize = 0;
        bool rawDataTruncated = false;
        uint8_t rawData[DEBUG_RAW_DATA_SIZE] = {0};
    };

private:
    HardwareSerial& _rp2040;
    static constexpr uint8_t kHeaderFirst = 'V';
    static constexpr uint8_t kHeaderSecond = 'L';

    enum class ReceiveState : uint8_t
    {
        WaitForV,
        WaitForL,
        ReceivePayload,
        ReceiveChecksum,
    };

    ReceiveState _receiveState = ReceiveState::WaitForV;
    uint8_t _payload[NUM_SENSORS * sizeof(uint16_t)] = {0};
    size_t _payloadIndex = 0;

    static uint8_t crc8(const uint8_t* data, size_t len);
    bool receiveDistance(uint16_t out[NUM_SENSORS], ReceiveDebugInfo* debugInfo);

public:
    uint16_t distance_data[NUM_SENSORS] = {0};

    UpperVL53s(HardwareSerial& rp2040) : _rp2040(rp2040) {}

    void getVL53s(uint16_t distance_data[NUM_SENSORS], ReceiveDebugInfo* debugInfo = nullptr);
};
}
