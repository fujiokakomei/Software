#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

namespace
{

constexpr uint8_t kTcaAddress = 0x70;
constexpr uint8_t kSensorCount = 3;
constexpr uint8_t kLeftSensor = 0;
constexpr uint8_t kCenterSensor = 1;
constexpr uint8_t kRightSensor = 2;
constexpr uint8_t kTcaChannels[kSensorCount] = {5, 6, 7};
constexpr uint16_t kContinuousPeriodMs = 30;
constexpr float kInvalidDistanceMm = -1.0f;

Adafruit_VL53L0X vl53[kSensorCount];
bool sensorReady[kSensorCount] = {};

const char *sensorName(uint8_t sensor)
{
    static constexpr const char *kSensorNames[kSensorCount] = {"left", "center", "right"};
    return sensor < kSensorCount ? kSensorNames[sensor] : "unknown";
}

bool selectTcaChannel(uint8_t channel)
{
    Wire.beginTransmission(kTcaAddress);
    Wire.write(static_cast<uint8_t>(1U << channel));
    const uint8_t status = Wire.endTransmission();

    Serial.print("[TCA] select channel ");
    Serial.print(channel);
    Serial.print(" (mask=0x");
    Serial.print(static_cast<uint8_t>(1U << channel), HEX);
    Serial.print("): I2C status=");
    Serial.println(status);

    return status == 0;
}

bool disableTca()
{
    Wire.beginTransmission(kTcaAddress);
    Wire.write(0x00);
    const uint8_t status = Wire.endTransmission();
    if (status != 0)
    {
        Serial.print("[TCA] disable failed: I2C status=");
        Serial.println(status);
    }
    return status == 0;
}

float readDistance(uint8_t sensor)
{
    Serial.print("[VL53] read ");
    Serial.print(sensorName(sensor));
    Serial.print(" (index=");
    Serial.print(sensor);
    Serial.println(")");

    if (sensor >= kSensorCount || !sensorReady[sensor])
    {
        Serial.println("  skipped: sensor is not ready");
        return kInvalidDistanceMm;
    }

    if (!selectTcaChannel(kTcaChannels[sensor]))
    {
        Serial.println("  failed: could not select TCA channel");
        return kInvalidDistanceMm;
    }

    if (!vl53[sensor].isRangeComplete())
    {
        const bool timedOut = vl53[sensor].timeoutOccurred();
        Serial.print("  measurement not ready");
        if (timedOut)
        {
            Serial.print(" (VL53 timeout)");
        }
        Serial.println();
        disableTca();
        return kInvalidDistanceMm;
    }

    const uint16_t distanceMm = vl53[sensor].readRangeResult();
    const uint8_t rangeStatus = vl53[sensor].readRangeStatus();
    const bool timedOut = vl53[sensor].timeoutOccurred();
    const bool valid = !timedOut && distanceMm < 8190;

    Serial.print("  raw_mm=");
    Serial.print(distanceMm);
    Serial.print(", range_status=");
    Serial.print(rangeStatus);
    Serial.print(", timeout=");
    Serial.print(timedOut ? "yes" : "no");
    Serial.print(", result=");
    Serial.println(valid ? "valid" : "invalid");

    disableTca();

    return valid ? static_cast<float>(distanceMm) : kInvalidDistanceMm;
}

} // namespace

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("=== Front VL53L0X debug test ===");
    Serial.print("I2C: TCA9548A=0x");
    Serial.print(kTcaAddress, HEX);
    Serial.print(", clock=400 kHz, continuous period=");
    Serial.print(kContinuousPeriodMs);
    Serial.println(" ms");

    Wire.begin();
    Wire.setClock(400000);

    for (uint8_t sensor = 0; sensor < kSensorCount; sensor++)
    {
        Serial.print("[VL53] initialize ");
        Serial.print(sensorName(sensor));
        Serial.print(" (TCA channel ");
        Serial.print(kTcaChannels[sensor]);
        Serial.println(")");

        if (!selectTcaChannel(kTcaChannels[sensor]))
        {
            sensorReady[sensor] = false;
            Serial.println("  initialization skipped: could not select TCA channel");
            continue;
        }

        const bool found = vl53[sensor].begin(VL53L0X_I2C_ADDR, false, &Wire);
        Serial.print("  begin(0x");
        Serial.print(VL53L0X_I2C_ADDR, HEX);
        Serial.print("): ");
        Serial.println(found ? "OK" : "FAILED");

        sensorReady[sensor] = found;
        if (sensorReady[sensor])
        {
            const bool started = vl53[sensor].startRangeContinuous(kContinuousPeriodMs);
            sensorReady[sensor] = started;
            Serial.print("  start continuous ranging: ");
            Serial.println(started ? "OK" : "FAILED");
        }

        disableTca();
    }

    Serial.println("[VL53] initialization summary");
    for (uint8_t sensor = 0; sensor < kSensorCount; sensor++)
    {
        Serial.print("  ");
        Serial.print(sensorName(sensor));
        Serial.print(": ");
        Serial.println(sensorReady[sensor] ? "ready" : "not ready");
    }
}

void loop()
{
    static uint32_t loopCount = 0;
    Serial.print("\n--- measurement #");
    Serial.print(++loopCount);
    Serial.print(" at ");
    Serial.print(millis());
    Serial.println(" ms ---");

    const float left = readDistance(kLeftSensor);
    const float center = readDistance(kCenterSensor);
    const float right = readDistance(kRightSensor);

    Serial.print("[SUMMARY] left=");
    Serial.print(left);
    Serial.print(" mm, center=");
    Serial.print(center);
    Serial.print(" mm, right=");
    Serial.print(right);
    Serial.println(" mm");

    delay(100);
}
