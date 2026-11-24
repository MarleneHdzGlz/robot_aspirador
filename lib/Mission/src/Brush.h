#pragma once
#include <Arduino.h>
#include "roomba_config.h"

class Brush {
public:
    void begin() {
        ledcSetup(BRUSH_LEDC_CH, BRUSH_PWM_FREQ, BRUSH_PWM_BITS);
        ledcAttachPin(PIN_BRUSH_RPWM, BRUSH_LEDC_CH);
        pinMode(PIN_BRUSH_EN, OUTPUT);
        digitalWrite(PIN_BRUSH_EN, LOW);
        ledcWrite(BRUSH_LEDC_CH, 0);
    }

    void setDuty(uint8_t duty) {  // 0..255
        digitalWrite(PIN_BRUSH_EN, duty > 0 ? HIGH : LOW);
        ledcWrite(BRUSH_LEDC_CH, duty);
    }

    void on()  { setDuty(255); }
    void off() { setDuty(0); }
};