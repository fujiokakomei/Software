#include <RescueKits.hpp>

namespace rescuekits
{

void RescueKitManager::setupRescueKits(int ServoPin, int ledPin, int rescuekits)
{
    this->ServoPin = ServoPin;
    this->ledPin = ledPin;

    this->remainingKits = rescuekits;

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //LEDを消灯

    //サーボモーターの初期化
    servo.attach(ServoPin);
    servo.write(0); //右のサーボモーターを初期位置に設定
}

void RescueKitManager::deployRescueKit(int number, bool is_led) //残りキットはmain.cppで確認
{
    for (int i = 0; i < number; i++)
    {
        servo.write(70); //右のサーボモーターを展開位置に設定
        delay(500);         //展開動作の完了を待つ
        servo.write(0); //右のサーボモーターを元の位置に戻す
        delay(500);
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