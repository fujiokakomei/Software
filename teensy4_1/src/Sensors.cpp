#include "Sensors.hpp"

#include <Wire.h>
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

constexpr unsigned long kGyroCalibrationTimeoutMs = 15000;
constexpr uint16_t kBlackClearThreshold = 120;
constexpr float kBlackLuxThreshold = 5.0f;
constexpr float kColorDominanceRatio = 1.3f;
constexpr uint16_t kSilverClearThreshold = 500;
constexpr float kSilverLuxThreshold = 30.0f;
constexpr float kSilverBalanceRatio = 1.2f;

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

} // namespace

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);
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


    Wire.begin();  // SDA/SCL: TCS34725 / BNO055
    Wire.setClock(400000); // I2Cのクロックを400kHzに設定

    _tcsReady = tcs.begin(TCS34725_ADDRESS, &Wire);
    if (!_tcsReady)
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

void SensorManager::readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c, float &colorTemp, float &lux)
{
    if (!_tcsReady)
    {
        r = g = b = c = 0;
        colorTemp = 0.0f;
        lux = 0.0f;
        return;
    }

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);
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
