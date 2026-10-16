#include <Arduino.h>

#include "Buttons.hpp"

int startb = 37;
int spareb1 = 36;
int spareb2 = 33;

buttons::ButtonManager startButton;
buttons::ButtonManager spare1Button;
buttons::ButtonManager spare2Button;

bool start;
bool spare1;
bool spare2;

void setup()
{
    startButton.setupButton(startb);
    spare1Button.setupButton(spareb1);
    spare2Button.setupButton(spareb2);
    Serial.begin(115200);
}

void loop()
{
    start = startButton.isPressed();
    spare1 = spare1Button.isPressed();
    spare2 = spare2Button.isPressed();
    if (start == true)
    {
        Serial.println("start pressed");
    }
    if (spare1 == true)
    {
        Serial.println("spare1 pressed");
    }
    if (spare2 == true)
    {
        Serial.println("spare2 pressed");
    }
}