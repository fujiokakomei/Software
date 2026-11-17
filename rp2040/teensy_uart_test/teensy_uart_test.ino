#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

namespace
{
constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kUartBaud = 115200;
constexpr uint32_t kUsbSerialWaitMs = 2000;
constexpr size_t kLineBufferSize = 64;

char receiveLine[kLineBufferSize] = {};
size_t receiveLineLength = 0;
uint32_t receivedPings = 0;

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

bool parsePing(const char* line, uint32_t& sequence)
{
    constexpr char kPrefix[] = "PING,";
    if (strncmp(line, kPrefix, sizeof(kPrefix) - 1) != 0)
    {
        return false;
    }

    const char* sequenceStart = line + sizeof(kPrefix) - 1;
    char* end = nullptr;
    const unsigned long parsedSequence = strtoul(sequenceStart, &end, 10);
    if (end == sequenceStart || *end != '\0')
    {
        return false;
    }

    sequence = static_cast<uint32_t>(parsedSequence);
    return true;
}

void sendPong(uint32_t sequence)
{
    Serial1.print(F("PONG,"));
    Serial1.print(sequence);
    Serial1.print(',');
    Serial1.println(millis());
}

void handleTeensyLine(const char* line)
{
    uint32_t sequence = 0;
    if (!parsePing(line, sequence))
    {
        Serial.print(F("RX unexpected: "));
        Serial.println(line);
        return;
    }

    ++receivedPings;
    sendPong(sequence);

    Serial.print(F("RX PING seq="));
    Serial.print(sequence);
    Serial.print(F(", sent PONG, count="));
    Serial.println(receivedPings);
}
} // namespace

void setup()
{
    Serial.begin(kUsbBaud);
    // XIAO RP2040: Serial1 TX=D7, RX=D8
    Serial1.begin(kUartBaud);

    const uint32_t waitStartedAt = millis();
    while (!Serial && millis() - waitStartedAt < kUsbSerialWaitMs)
    {
    }

    Serial.println(F("RP2040 <-> Teensy UART test started"));
    Serial.println(F("Serial1: 115200 baud, TX=D7, RX=D8"));
    Serial.println(F("Waiting for PING messages from Teensy..."));
}

void loop()
{
    char line[kLineBufferSize] = {};
    while (readLine(Serial1, line, sizeof(line)))
    {
        handleTeensyLine(line);
    }
}
