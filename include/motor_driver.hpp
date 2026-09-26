#pragma once
#include <Arduino.h>

class MotorDriver {

public:
    bool begin();
    void write(float pwm_R, float pwm_L);
    void stop();
    
private:
    // Positive PWM drives the forward pin, negative PWM drives the backward pin.
    void writeMotor(int forward_pin,
                int backward_pin,
                int forward_pwm_channel,
                int backward_pwm_channel,
                float pwm);
    // Arduino-ESP32 2.x and 3.x expose slightly different LEDC APIs. The implementation
    // hides that difference from the rest of the project.
    void writeDuty(int output_pin, int pwm_channel, uint32_t duty);
};
