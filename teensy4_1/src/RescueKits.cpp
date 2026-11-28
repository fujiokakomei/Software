#include <RescueKits.hpp>

namespace rescuekits
{

namespace
{
constexpr unsigned long kDeployHoldMs = 500;  // 投下位置で待つ時間
constexpr unsigned long kReturnWaitMs = 500;  // 普段の角度に戻るまで待つ時間
}

void RescueKitManager::setupRescueKits(int ServoPin, int ledPin, int rescuekits, int idleAngle, int deployAngle)
{
    this->ServoPin = ServoPin;
    this->ledPin = ledPin;
    this->idleAngle = idleAngle;
    this->deployAngle = deployAngle;

    this->remainingKits = rescuekits;

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //LEDを消灯

    //サーボモーターの初期化
    servo.attach(ServoPin);
    servo.write(idleAngle); //普段の角度に設定
}

void RescueKitManager::deployRescueKit(int number, bool is_led) //残りキットはmain.cppで確認
{
    for (int i = 0; i < number; i++)
    {
        servo.write(deployAngle); //投下時の角度にする
        delay(kDeployHoldMs);     //投下動作の完了を待つ
        servo.write(idleAngle);   //普段の角度に戻す
        delay(kReturnWaitMs);
        remainingKits--;
    }

    if (is_led == true)
    {
        for (int i = 0; i < number; i++)
        {
            for (int i = 0; i < 5; i++)
            {
                digitalWrite(ledPin, HIGH); //LEDを点灯
                delay(300);
                digitalWrite(ledPin, LOW); //LEDを消灯
                delay(300);
            }
        }
    }
}

} // namespace rescuekits
