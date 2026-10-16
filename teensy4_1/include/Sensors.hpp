#pragma once

#include <Arduino.h>

namespace sensors
{

#define TCA_ADDR 0x70 // SDA1/SCL1(Wire1)上のTCA9548A

enum class ColorTile
{
    None,
    Blue,
    Red,
    Silver,
    Black
};

class SensorManager
{
public:

    static void tcaSelect(uint8_t ch);
    static void tcaDisable();


    // 全センサーの初期化とジャイロ校正が成功した場合のみtrueを返す。
    bool setupSensors(int led_pin);
    bool isReady() const;

    //単位はmm
    float readRightDistance(); //右前(1つ)
    float readLeftDistance(); //左前(1つ)
    float readFrontDistance(); //前3つの平均
    //float readBackDistance(); //後ろのVL53は未搭載のため-1.0f
    float readRightFrontDistance(); //右前(1つ)
    float readCenterFrontDistance(); //中央前(1つ)
    float readLeftFrontDistance(); //左前(1つ)
    //float readRightBackDistance(); //未搭載のため-1.0f
    //float readLeftBackDistance(); //未搭載のため-1.0f

    int led_pin;
    void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c, float &colorTemp, float &lux);

    float yawoffset = 0.0f; //ジャイロセンサーのヨー軸オフセット値、定期的に更新
    float rolloffset = 0.0f; //ジャイロセンサーのロール軸オフセット値、更新なし(ただの初期値)
    float pitchoffset = 0.0f; //ジャイロセンサーのピッチ軸オフセット値、更新なし(ただの初期値)
    bool waitForGyroCalibration(); //起動時に実行、タイムアウト時はfalse
    bool calibrateGyroOffset(int nowDir); //ジャイロセンサーのオフセット値をキャリブレーションする関数
    void readOrientation(float &x, float &y, float &z);
    float readYaw();

private:
    bool _isReady = false;
    bool _tcsReady = false;
    bool _bnoReady = false;
};

ColorTile detectColorTile(SensorManager& sensors);

}
