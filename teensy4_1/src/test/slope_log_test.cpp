// 坂判定（SlopeDetector）の調整用。モーターは回さない。
// 機体を手で押して坂・階段・バンプの上を通し、シリアルモニタ(115200)で
// 前後傾き・左右傾き・判定状態を確認する。CSVなのでシリアルプロッタにも出せる。
//   pio run -e slope_log_test -t upload
//
// 確認すること:
//   - 機首を上げたとき pitch が正になるか（逆なら Sensors.cpp の kNoseUpSign を -1 に、
//     増えない場合は kUseRollAxisForPitch を切り替える）
//   - 坂では state が 2(OnSlope) → 3(Finished) になり、階段・バンプでは 3 にならないか
//   - 段差やバンプでの最大 pitch が enterPitchDeg(12度) より十分小さいか
#include <Arduino.h>

#include "Sensors.hpp"
#include "SlopeDetector.hpp"

namespace
{
constexpr int kTcsLedPin = 40;
constexpr int kStartButtonPin = 37;
// 本番のmoveForwardOneBlockはカラーセンサー読み取りを含めて1周約50msなので合わせる
constexpr unsigned long kSampleIntervalMs = 50;

sensors::SensorManager sensorManager;
slope::SlopeDetector detector;
bool ready = false;

const char* stateName(slope::SlopeDetector::State state)
{
    switch (state)
    {
    case slope::SlopeDetector::State::Flat: return "Flat";
    case slope::SlopeDetector::State::Candidate: return "Candidate";
    case slope::SlopeDetector::State::OnSlope: return "OnSlope";
    case slope::SlopeDetector::State::Finished: return "Finished";
    case slope::SlopeDetector::State::Rejected: return "Rejected";
    }
    return "?";
}
} // namespace

void setup()
{
    Serial.begin(115200);
    pinMode(kStartButtonPin, INPUT_PULLUP);
    delay(1000);
    Serial.println("slope_log_test: 機体を水平に置いて待機（ジャイロ校正）");

    ready = sensorManager.setupSensors(kTcsLedPin);
    if (!ready)
    {
        Serial.println("センサーの初期化に失敗（TCS34725 / BNO055 の配線、BNO055の校正）");
        return;
    }
    Serial.println("ready. スタートボタンで判定をリセット");
    Serial.println("ms,pitch,roll,state,direction");
}

void loop()
{
    if (!ready)
    {
        delay(1000);
        return;
    }

    if (digitalRead(kStartButtonPin) == LOW)
    {
        detector.reset();
        Serial.println("# reset");
        delay(300);
    }

    float pitch = 0.0f;
    float roll = 0.0f;
    sensorManager.readTilt(pitch, roll);
    const unsigned long now = millis();
    const slope::SlopeDetector::State before = detector.state();
    detector.update(pitch, roll, now);

    Serial.print(now);
    Serial.print(',');
    Serial.print(pitch, 1);
    Serial.print(',');
    Serial.print(roll, 1);
    Serial.print(',');
    Serial.print(static_cast<int>(detector.state()));
    Serial.print(',');
    Serial.println(detector.direction());
    if (detector.state() != before)
    {
        Serial.print("# -> ");
        Serial.println(stateName(detector.state()));
    }
    if (detector.isFinished())
    {
        Serial.println(detector.direction() > 0 ? "# 上り坂を通過" : "# 下り坂を通過");
        detector.reset();
    }

    delay(kSampleIntervalMs);
}
