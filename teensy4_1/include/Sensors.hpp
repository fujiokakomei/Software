#pragma once

#include <Arduino.h>

namespace sensors
{

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
    // 全センサーの初期化とジャイロ校正が成功した場合のみtrueを返す。
    bool setupSensors(int led_pin);
    bool isReady() const;

    int led_pin;
    void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c, float &colorTemp, float &lux);

    float yawoffset = 0.0f; //ジャイロセンサーのヨー軸オフセット値、定期的に更新
    float rolloffset = 0.0f; //ジャイロセンサーのロール軸オフセット値、更新なし(ただの初期値)
    float pitchoffset = 0.0f; //ジャイロセンサーのピッチ軸オフセット値、更新なし(ただの初期値)
    bool waitForGyroCalibration(); //起動時に実行、タイムアウト時はfalse
    bool calibrateGyroOffset(int nowDir); //ジャイロセンサーのオフセット値をキャリブレーションする関数
    void readOrientation(float &x, float &y, float &z);
    // 坂判定用。起動時を0度として、機首上げを正とした前後の傾きと左右の傾き[deg]を返す。
    void readTilt(float &noseUpPitch, float &sideRoll);
    float readYaw();

private:
    bool _isReady = false;
    bool _tcsReady = false;
    bool _bnoReady = false;
};

ColorTile detectColorTile(SensorManager& sensors);

}
