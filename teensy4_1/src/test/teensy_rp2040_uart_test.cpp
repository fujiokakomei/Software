#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

namespace
{
constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kUartBaud = 115200;
constexpr uint32_t kPingIntervalMs = 1000;
constexpr uint32_t kReplyTimeoutMs = 1500;
constexpr uint32_t kUsbSerialWaitMs = 2000;
constexpr size_t kLineBufferSize = 64;

char receiveLine[kLineBufferSize] = {};
size_t receiveLineLength = 0;
uint32_t nextSequence = 1;
uint32_t pendingSequence = 0;
uint32_t pingSentAt = 0;
uint32_t lastPingAt = 0;
uint32_t successfulReplies = 0;
uint32_t timedOutReplies = 0;
bool waitingForReply = false;

// 改行区切りのUARTメッセージを1行ずつ取り出す。
bool readLine(Stream& serial, char* line, size_t lineSize)
{
    while (serial.available() > 0)
    {
        const char character = static_cast<char>(serial.read());
        if (character == '\r')
        {
            continue;
        }

        if (character == '\n')
        {
            receiveLine[receiveLineLength] = '\0';
            strncpy(line, receiveLine, lineSize);
            line[lineSize - 1] = '\0';
            receiveLineLength = 0;
            return true;
        }

        if (receiveLineLength < sizeof(receiveLine) - 1)
        {
            receiveLine[receiveLineLength++] = character;
        }
        else
        {
            // 壊れた行を捨て、次の改行から再同期する。
            receiveLineLength = 0;
        }
    }
    return false;
}

bool parsePong(const char* line, uint32_t& sequence, uint32_t& rp2040Millis)
{
    constexpr char kPrefix[] = "PONG,";
    if (strncmp(line, kPrefix, sizeof(kPrefix) - 1) != 0)
    {
        return false;
    }

    const char* sequenceStart = line + sizeof(kPrefix) - 1;
    char* end = nullptr;
    const unsigned long parsedSequence = strtoul(sequenceStart, &end, 10);
    if (end == sequenceStart || *end != ',')
    {
        return false;
    }

    const char* millisStart = end + 1;
    const unsigned long parsedMillis = strtoul(millisStart, &end, 10);
    if (end == millisStart || *end != '\0')
    {
        return false;
    }

    sequence = static_cast<uint32_t>(parsedSequence);
    rp2040Millis = static_cast<uint32_t>(parsedMillis);
    return true;
}

void sendPing()
{
    pendingSequence = nextSequence++;
    pingSentAt = millis();
    lastPingAt = pingSentAt;
    waitingForReply = true;

    Serial8.print(F("PING,"));
    Serial8.println(pendingSequence);

    Serial.print(F("TX PING seq="));
    Serial.println(pendingSequence);
}

void handlePeerLine(const char* line)
{
    uint32_t sequence = 0;
    uint32_t rp2040Millis = 0;
    if (!parsePong(line, sequence, rp2040Millis))
    {
        Serial.print(F("RX unexpected: "));
        Serial.println(line);
        return;
    }

    if (!waitingForReply || sequence != pendingSequence)
    {
        Serial.print(F("RX PONG with unexpected seq="));
        Serial.println(sequence);
        return;
    }

    const uint32_t roundTripMs = millis() - pingSentAt;
    waitingForReply = false;
    ++successfulReplies;

    Serial.print(F("RX PONG seq="));
    Serial.print(sequence);
    Serial.print(F(", RP2040 t="));
    Serial.print(rp2040Millis);
    Serial.print(F(" ms, RTT="));
    Serial.print(roundTripMs);
    Serial.print(F(" ms, OK="));
    Serial.println(successfulReplies);
}
} // namespace

void setup()
{
    Serial.begin(kUsbBaud);
    Serial8.begin(kUartBaud);

    const uint32_t waitStartedAt = millis();
    while (!Serial && millis() - waitStartedAt < kUsbSerialWaitMs)
    {
    }

    Serial.println(F("Teensy <-> RP2040 UART test started"));
    Serial.println(F("Serial8: 115200 baud, RX=pin 34, TX=pin 35"));
    Serial.println(F("Waiting for PONG replies from RP2040..."));
}

void loop()
{
    char line[kLineBufferSize] = {};
    while (readLine(Serial8, line, sizeof(line)))
    {
        handlePeerLine(line);
    }

    const uint32_t now = millis();
    if (waitingForReply && now - pingSentAt >= kReplyTimeoutMs)
    {
        ++timedOutReplies;
        Serial.print(F("TIMEOUT seq="));
        Serial.print(pendingSequence);
        Serial.print(F(", timeout count="));
        Serial.println(timedOutReplies);
        waitingForReply = false;
    }

    if (!waitingForReply && now - lastPingAt >= kPingIntervalMs)
    {
        sendPing();
    }
}
