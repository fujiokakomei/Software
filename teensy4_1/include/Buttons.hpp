#pragma once
#include <Arduino.h>

namespace buttons
{
class ButtonManager
{
public:
    int pin;
    int lastState = HIGH; //前回の状態を保存する変数、初期値はHIGH（未押下）
    void setupButton(int pin);
    bool isPressed();
};

// スタートボタンの割り込みを設定する（チャタリング除去付き）。
void setupStartButtonInterrupt(int pin);
// 受け付け済みの押下を捨てる
void clearStartButtonInterrupt();
void requestStartButtonInterrupt();
bool consumeStartButtonInterrupt();
bool hasStartButtonInterrupt();
bool takeConsumedStartButtonInterrupt();
} // namespace buttons
