#include "Sensors.hpp"

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

/*
    Front

    D2 --- D1 --- D0
    Left  Center Right
*/

namespace sensors
{

namespace
{

constexpr uint8_t kVl53Count = 3;
constexpr uint16_t kVl53ContinuousPeriodMs = 30;
constexpr float kInvalidDistanceMm = -1.0f;
constexpr unsigned long kGyroCalibrationTimeoutMs = 15000;
constexpr uint16_t kBlackClearThreshold = 120;
constexpr float kBlackLuxThreshold = 5.0f;
constexpr float kColorDominanceRatio = 1.3f;
constexpr uint16_t kSilverClearThreshold = 500;
constexpr float kSilverLuxThreshold = 30.0f;
constexpr float kSilverBalanceRatio = 1.2f;

constexpr uint8_t kRightFrontDistance = 0;
constexpr uint8_t kCenterFrontDistance = 1;
constexpr uint8_t kLeftFrontDistance = 2;

float normalizeYaw360(float yaw)
{
    while (yaw >= 360.0f)
    {
        yaw -= 360.0f;
    }
    while (yaw < 0.0f)
    {
        yaw += 360.0f;
    }
    return yaw;
}

Adafruit_VL53L0X vl[kVl53Count];                 // Ch.0〜2
float vlDistanceMm[kVl53Count] = {
    kInvalidDistanceMm,
    kInvalidDistanceMm,
    kInvalidDistanceMm
};
bool vlAvailable[kVl53Count] = {};

bool updateVl53Distance(uint8_t ch)
{
    if (ch >= kVl53Count)
    {
        return false;
    }

    SensorManager::tcaSelect(ch);
    if (vl[ch].isRangeComplete())
    {
        const uint16_t distanceMm = vl[ch].readRangeResult();
        if (!vl[ch].timeoutOccurred() && distanceMm < 8190)
        {
            vlDistanceMm[ch] = static_cast<float>(distanceMm);
            vlAvailable[ch] = true;
        }
        else
        {
            vlDistanceMm[ch] = kInvalidDistanceMm;
            vlAvailable[ch] = false;
        }
    }
    SensorManager::tcaDisable();

    return vlAvailable[ch];
}

float readVl53Distance(uint8_t ch)
{
    updateVl53Distance(ch);
    return vlDistanceMm[ch];
}

float averageFrontDistance()
{
    float totalDistanceMm = 0.0f;
    uint8_t availableCount = 0;

    for (uint8_t i = 0; i < kVl53Count; i++)
    {
        if (updateVl53Distance(i))
        {
            totalDistanceMm += vlDistanceMm[i];
            availableCount++;
        }
    }

    return availableCount > 0 ? totalDistanceMm / availableCount : kInvalidDistanceMm;
}

} // namespace

void SensorManager::tcaSelect(uint8_t ch)
{
    if (ch > 7)
    {
        return;
    }
    Wire1.beginTransmission(TCA_ADDR);
    Wire1.write(1 << ch);
    Wire1.endTransmission();
}

void SensorManager::tcaDisable() {
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(0x00);
  Wire1.endTransmission();
}


Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);                                                 // Ch.3
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // SDA/SCLに直結 (ADR→GND)

bool SensorManager::setupSensors(int led_pin)
{
    _isReady = false;
    _tcsReady = false;
    _bnoReady = false;

    //TCS34725のLEDセットアップ
    this->led_pin = led_pin;
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, HIGH);


    Wire.begin();  // SDA/SCL: BNO055
    Wire.setClock(400000); // I2Cのクロックを400kHzに設定
    Wire1.begin(); // SDA1/SCL1: TCA9548A配下のVL53L0X/TCS34725
    Wire1.setClock(400000); // I2C1のクロックを400kHzに設定

    bool allVl53Ready = true;
    for (uint8_t i = 0; i < kVl53Count; i++) // Ch.0〜2の距離センサーを初期化
    {
        SensorManager::tcaSelect(i);
        if (vl[i].begin(VL53L0X_I2C_ADDR, false, &Wire1))
        {
            vl[i].startRangeContinuous(kVl53ContinuousPeriodMs);
            vlDistanceMm[i] = kInvalidDistanceMm;
            vlAvailable[i] = false;
        }
        else
        {
            allVl53Ready = false;
            vlDistanceMm[i] = kInvalidDistanceMm;
            vlAvailable[i] = false;
        }
    }

    SensorManager::tcaSelect(3); // Ch.3のカラーセンサーを初期化
    _tcsReady = tcs.begin(TCS34725_ADDRESS, &Wire1);

    SensorManager::tcaDisable(); // TCAを無効化
    if (!allVl53Ready || !_tcsReady)
    {
        digitalWrite(led_pin, LOW);
        return false;
    }

    _bnoReady = bno.begin();
    if (!_bnoReady)
    {
        digitalWrite(led_pin, LOW);
        return false;
    }
    bno.setExtCrystalUse(true); //外部クロック有効
    bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS); //加速度センサーとジャイロセンサーのみ有効
    delay(100); //センサーが安定するまで待機
    if (!waitForGyroCalibration())
    {
        digitalWrite(led_pin, LOW);
        return false;
    }

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    yawoffset = event.orientation.x; //現在のyawをオフセット値として保存
    rolloffset = event.orientation.y; //現在のrollをオフセット値として保存
    pitchoffset = event.orientation.z; //現在のpitchをオフセット値として保存
    _isReady = true;
    return true;
}

bool SensorManager::isReady() const
{
    return _isReady;
}

float SensorManager::readRightDistance()
{
    return readRightFrontDistance();
}
float SensorManager::readLeftDistance()
{
    return readLeftFrontDistance();
}
float SensorManager::readFrontDistance()
{
    return averageFrontDistance();
}

/*
float SensorManager::readBackDistance()
{
    return kInvalidDistanceMm;
}
*/

float SensorManager::readRightFrontDistance()
{
    return readVl53Distance(kRightFrontDistance);
}

float SensorManager::readCenterFrontDistance()
{
    return readVl53Distance(kCenterFrontDistance);
}

float SensorManager::readLeftFrontDistance()
{
    return readVl53Distance(kLeftFrontDistance);
}

/*
float SensorManager::readRightBackDistance()
{
    return kInvalidDistanceMm;
}

float SensorManager::readLeftBackDistance()
{
    return kInvalidDistanceMm;
}
*/

void SensorManager::readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c, float &colorTemp, float &lux)
{
    if (!_tcsReady)
    {
        r = g = b = c = 0;
        colorTemp = 0.0f;
        lux = 0.0f;
        return;
    }

    SensorManager::tcaSelect(3); // Ch.3のカラーセンサーを選択
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);
    SensorManager::tcaDisable();
}

ColorTile detectColorTile(SensorManager& sensors)
{
    if (!sensors.isReady())
    {
        return ColorTile::None;
    }

    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;
    uint16_t c = 0;
    float colorTemp = 0.0f;
    float lux = 0.0f;
    sensors.readColor(r, g, b, c, colorTemp, lux);

    if (c < kBlackClearThreshold || lux < kBlackLuxThreshold)
    {
        return ColorTile::Black;
    }

    const uint16_t maxRgb = max(r, max(g, b));
    const uint16_t minRgb = min(r, min(g, b));
    if (c > kSilverClearThreshold
        && lux > kSilverLuxThreshold
        && static_cast<float>(maxRgb) < static_cast<float>(minRgb) * kSilverBalanceRatio)
    {
        return ColorTile::Silver;
    }

    if (r > g * kColorDominanceRatio && r > b * kColorDominanceRatio)
    {
        return ColorTile::Red;
    }

    if (b > r * kColorDominanceRatio && b > g * kColorDominanceRatio)
    {
        return ColorTile::Blue;
    }

    return ColorTile::None;
}


/*
x: Yaw 0~360
y: Roll -90~90
z: Pitch -180~180
*/

bool SensorManager::waitForGyroCalibration()
{
    if (!_bnoReady)
    {
        return false;
    }

    const unsigned long startTime = millis();
    while (millis() - startTime < kGyroCalibrationTimeoutMs)
    {
        uint8_t sysCal, gyroCal, accelCal, magCal;
        bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
        if (gyroCal == 3) // ジャイロセンサーのキャリブレーションが完了しているか
        {
            return true;
        }
        delay(100);
    }
    return false;
}

bool SensorManager::calibrateGyroOffset(int nowDir) //現在のyawをオフセットに
{
    if (!_isReady)
    {
        return false;
    }

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER); //オイラー角で取得
    //nowDirから現在のmapper上での方角を取得
    yawoffset = event.orientation.x - nowDir * 90.0f; //現在のyawからnowDirを引いてオフセットにする
    return true;
}

void SensorManager::readOrientation(float &x, float &y, float &z)
{
    if (!_bnoReady)
    {
        x = y = z = 0.0f;
        return;
    }

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER); //オイラー角で取得
    x = normalizeYaw360(event.orientation.x - yawoffset); // setupSensors実行時の向きを0度として0〜360度で返す
    y = event.orientation.y - rolloffset; //初期値からの差分を返す
    z = event.orientation.z - pitchoffset; //初期値からの差分を返す
}

float SensorManager::readYaw()
{
    float x, y, z;
    SensorManager::readOrientation(x, y, z);
    return x;
}

} // namespace sensors
