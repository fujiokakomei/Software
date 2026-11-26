// UnitV AI Camera と Teensy 4.1 の UART 疎通テスト。
//
// 配線（既存の UnitV 1 と同じ Serial3 を使う）:
//   Teensy pin 14 (Serial3 TX) -> UnitV GPIO34 (UART1 RX)
//   Teensy pin 15 (Serial3 RX) <- UnitV GPIO35 (UART1 TX)
//   Teensy GND                 --- UnitV GND
//
// unitv/camera_uart_test.py と同時に動かすと、Teensy は1秒ごとに
// "ping" を送り、UnitV からの "RX: ping"、カメラ状態、起動通知を
// USBシリアル（115200 bps）へ表示する。

#include <Arduino.h>
#include <string.h>

namespace
{
constexpr uint32_t kBaudrate = 115200;
constexpr unsigned long kPingIntervalMs = 1000;
constexpr unsigned long kHealthIntervalMs = 3000;
constexpr size_t kLineBufferSize = 192;

// UnitV 2 を試す場合は Serial5 に変更する。
HardwareSerial &unitvSerial = Serial5;

char unitvLine[kLineBufferSize];
size_t unitvLineLength = 0;
char usbLine[kLineBufferSize];
size_t usbLineLength = 0;

bool unitvReady = false;
unsigned long lastEchoMs = 0;
unsigned long lastPingMs = 0;
unsigned long lastHealthMs = 0;

void sendToUnitV(const char *message)
{
    unitvSerial.println(message);
    Serial.print(F("Teensy > UnitV: "));
    Serial.println(message);
}

void handleUnitVLine()
{
    unitvLine[unitvLineLength] = '\0';
    Serial.print(F("UnitV > Teensy: "));
    Serial.println(unitvLine);

    if (strcmp(unitvLine, "UNITV_CAMERA_TEST_READY") == 0)
    {
        unitvReady = true;
        Serial.println(F("UnitV startup message confirmed."));
    }
    else if (strcmp(unitvLine, "RX: ping") == 0)
    {
        lastEchoMs = millis();
        Serial.println(F("UART round-trip confirmed."));
    }

    unitvLineLength = 0;
}

void readFromUnitV()
{
    while (unitvSerial.available() > 0)
    {
        const char received = static_cast<char>(unitvSerial.read());
        if (received == '\r')
        {
            continue;
        }
        if (received == '\n')
        {
            handleUnitVLine();
        }
        else if (unitvLineLength < kLineBufferSize - 1)
        {
            unitvLine[unitvLineLength++] = received;
        }
        else
        {
            Serial.println(F("UnitV receive line too long; discarding it."));
            unitvLineLength = 0;
        }
    }
}

void readFromUsb()
{
    // USBシリアルモニタから任意の1行を送ると、そのままUnitVに中継する。
    while (Serial.available() > 0)
    {
        const char received = static_cast<char>(Serial.read());
        if (received == '\r')
        {
            continue;
        }
        if (received == '\n')
        {
            if (usbLineLength > 0)
            {
                usbLine[usbLineLength] = '\0';
                sendToUnitV(usbLine);
                usbLineLength = 0;
            }
        }
        else if (usbLineLength < kLineBufferSize - 1)
        {
            usbLine[usbLineLength++] = received;
        }
        else
        {
            Serial.println(F("USB receive line too long; discarding it."));
            usbLineLength = 0;
        }
    }
}

void printHealthStatus()
{
    const unsigned long now = millis();
    const bool echoRecent = lastEchoMs != 0 && now - lastEchoMs < kHealthIntervalMs;

    Serial.print(F("STATUS ready="));
    Serial.print(unitvReady ? F("yes") : F("no"));
    Serial.print(F(" ping_echo="));
    Serial.println(echoRecent ? F("ok") : F("waiting"));
}
} // namespace

void setup()
{
    Serial.begin(kBaudrate);
    const unsigned long usbWaitStart = millis();
    while (!Serial && millis() - usbWaitStart < 3000)
    {
        // USBシリアルモニタの接続を最大3秒待つ。
    }

    unitvSerial.begin(kBaudrate);
    Serial.println(F("UnitV UART test started."));
    Serial.println(F("Send a line through USB to relay it to UnitV."));
    sendToUnitV("ping");
    lastPingMs = millis();
}

void loop()
{
    readFromUnitV();
    readFromUsb();

    const unsigned long now = millis();
    if (now - lastPingMs >= kPingIntervalMs)
    {
        sendToUnitV("ping");
        lastPingMs = now;
    }
    if (now - lastHealthMs >= kHealthIntervalMs)
    {
        printHealthStatus();
        lastHealthMs = now;
    }
}
