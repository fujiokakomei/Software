#pragma once

#include <Arduino.h>

namespace uppervl53s
{
class UpperVL53s
{
public:
    static constexpr size_t NUM_SENSORS = 12;
    static constexpr size_t PAYLOAD_SIZE = NUM_SENSORS * sizeof(uint16_t);
    // RP2040側でそのセンサーが起動できていない（測定値がない）ことを表す値。
    // 一度もフレームを受信していないときも全センサーこの値になる。
    // 0xFFFF は VL53 の「範囲外（遠すぎる）」。未使用のセンサーは 0。
    static constexpr uint16_t NO_DATA = 0xFFFE;
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
    static constexpr size_t FRAME_SIZE = 2 + PAYLOAD_SIZE + 1; // 'V','L', 距離, CRC
    // Serial8の標準受信バッファは64バイト(約2フレーム)しかなく、旋回やLED点滅などで
    // 読み出しが止まるとすぐ溢れる。溢れると新しい方が捨てられ古い値が残るので増やす。
    static constexpr size_t EXTRA_RX_BUFFER_SIZE = 1024; // 約37フレーム ≒ 2秒分
    static constexpr size_t STANDARD_RX_BUFFER_SIZE = 64;
    // 受信バッファがここまで溜まっていたら溢れて古いデータしか残っていないとみなす
    static constexpr size_t STALE_BUFFER_BYTES =
        STANDARD_RX_BUFFER_SIZE + EXTRA_RX_BUFFER_SIZE - 2 * FRAME_SIZE;
    // 古いデータを捨てた後、新しいフレームを待つ上限
    static constexpr unsigned long FRESH_FRAME_TIMEOUT_MS = 200;

    HardwareSerialIMXRT& _rp2040;
    uint8_t _extraRxBuffer[EXTRA_RX_BUFFER_SIZE];
    unsigned long _lastFrameMs = 0;
    bool _hasFrame = false;
    // 距離フレーム: 'V','L', 距離(12個 x 16bit little endian), CRC-8
    static constexpr uint8_t kHeaderFirst = 'V';
    static constexpr uint8_t kHeaderSecond = 'L';
    // 状況メッセージフレーム: 'M','G', 長さ, 文字列, CRC-8(長さ+文字列)
    static constexpr uint8_t kMessageHeaderFirst = 'M';
    static constexpr uint8_t kMessageHeaderSecond = 'G';
    static constexpr size_t MESSAGE_MAX_LENGTH = 200; // rp2040.ino と同じ値

    enum class ReceiveState : uint8_t
    {
        WaitForV,          // ヘッダ1文字目待ち（'V' または 'M'）
        WaitForL,
        ReceivePayload,
        ReceiveChecksum,
        WaitForG,
        ReceiveMessageLength,
        ReceiveMessage,
        ReceiveMessageChecksum,
    };

    uint8_t _message[1 + MESSAGE_MAX_LENGTH + 1] = {0}; // 長さ + 文字列 + 終端
    size_t _messageLength = 0;
    size_t _messageIndex = 0;
    unsigned long _messageCount = 0;

    void startHeader(uint8_t byte);
    void handleMessage();

    ReceiveState _receiveState = ReceiveState::WaitForV;
    uint8_t _payload[NUM_SENSORS * sizeof(uint16_t)] = {0};
    size_t _payloadIndex = 0;

    static uint8_t crc8(const uint8_t* data, size_t len);
    bool receiveDistance(uint16_t out[NUM_SENSORS], ReceiveDebugInfo* debugInfo);

public:
    uint16_t distance_data[NUM_SENSORS] = {
        NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA,
        NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};

    UpperVL53s(HardwareSerialIMXRT& rp2040) : _rp2040(rp2040) {}

    // シリアルを開始し、受信バッファを増やす。HardwareSerial::begin() の代わりに呼ぶ。
    void begin(uint32_t baud);
    void getVL53s(uint16_t distance_data[NUM_SENSORS], ReceiveDebugInfo* debugInfo = nullptr);
    // 最後に正しいフレームを受信してからの時間[ms]。一度も受信していなければULONG_MAX。
    unsigned long lastFrameAgeMs() const;
    // RP2040から受け取った状況メッセージ（最後の1件）と受信数
    const char* lastMessage() const { return reinterpret_cast<const char*>(_message + 1); }
    unsigned long messageCount() const { return _messageCount; }
};
}
