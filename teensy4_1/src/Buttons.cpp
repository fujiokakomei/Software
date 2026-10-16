#include "Buttons.hpp"

namespace buttons
{
namespace
{
volatile bool startButtonInterrupted = false;
volatile bool consumedStartButtonInterrupt = false;
constexpr unsigned long kStartButtonDebounceUs = 50000;
volatile unsigned long lastStartButtonInterruptUs = 0;
volatile bool hasStartButtonInterruptTimestamp = false;
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
    return pressed;
}

void requestStartButtonInterrupt()
{
    const unsigned long nowUs = micros();
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
