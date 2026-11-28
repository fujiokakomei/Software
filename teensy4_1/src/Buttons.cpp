#include "Buttons.hpp"

namespace buttons
{
namespace
{
volatile bool startButtonInterrupted = false;
volatile bool consumedStartButtonInterrupt = false;
// 押したと受け付けた後、次の押下を受け付けない時間
constexpr unsigned long kStartButtonDebounceUs = 300000;
// LOW になる直前に、これだけの間 HIGH（離した状態）が続いていなければチャタリングとみなす。
// ボタンを離すときのばたつき(HIGH→LOW→HIGH)を「押した」と誤検出しないため。
constexpr unsigned long kStartButtonReleasedStableUs = 30000;
volatile unsigned long lastStartButtonInterruptUs = 0;
volatile bool hasStartButtonInterruptTimestamp = false;
volatile unsigned long lastStartButtonHighUs = 0; // 最後に HIGH になった時刻
int startButtonPin = -1;
}

void setupStartButtonInterrupt(int pin)
{
    startButtonPin = pin;
    lastStartButtonHighUs = micros();
    // 押したときだけでなく離したときも割り込みを受けて、離した時刻を記録する
    attachInterrupt(digitalPinToInterrupt(pin), requestStartButtonInterrupt, CHANGE);
}

void clearStartButtonInterrupt()
{
    noInterrupts();
    startButtonInterrupted = false;
    consumedStartButtonInterrupt = false;
    interrupts();
}

void ButtonManager::setupButton(int pin)
{
    this->pin = pin;
    lastState = HIGH;
    pinMode(pin, INPUT_PULLUP); //プルアップ抵抗を有効にして入力モードに設定
}

bool ButtonManager::isPressed()
{
    int currentState = digitalRead(pin);
    bool pressed = (lastState == HIGH && currentState == LOW); //前回がHIGHで今回がLOWなら押されたと判断
    lastState = currentState; //現在の状態を保存
    delay(50);
    return pressed;
}

void requestStartButtonInterrupt()
{
    const unsigned long nowUs = micros();
    if (startButtonPin >= 0)
    {
        if (digitalReadFast(startButtonPin) == HIGH)
        {
            // 離した（またはばたつき中にHIGHになった）だけ
            lastStartButtonHighUs = nowUs;
            return;
        }
        if (nowUs - lastStartButtonHighUs < kStartButtonReleasedStableUs)
        {
            // HIGH になった直後の LOW はチャタリング
            return;
        }
    }

    if (hasStartButtonInterruptTimestamp &&
        nowUs - lastStartButtonInterruptUs < kStartButtonDebounceUs)
    {
        return;
    }

    lastStartButtonInterruptUs = nowUs;
    hasStartButtonInterruptTimestamp = true;
    startButtonInterrupted = true;
}

bool consumeStartButtonInterrupt()
{
    noInterrupts();
    const bool interrupted = startButtonInterrupted;
    startButtonInterrupted = false;
    if (interrupted)
    {
        consumedStartButtonInterrupt = true;
    }
    interrupts();
    return interrupted;
}

bool hasStartButtonInterrupt()
{
    noInterrupts();
    const bool interrupted = startButtonInterrupted;
    interrupts();
    return interrupted;
}

bool takeConsumedStartButtonInterrupt()
{
    noInterrupts();
    const bool consumed = consumedStartButtonInterrupt;
    consumedStartButtonInterrupt = false;
    interrupts();
    return consumed;
}
} // namespace buttons
