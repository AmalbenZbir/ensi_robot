#include "motor_driver.hpp"

#include <cmath>

#include "robot_config.hpp"

#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

bool MotorDriver::begin(){
    pinMode(robot_config::motor_R_forward_pin, OUTPUT);
    pinMode(robot_config::motor_R_backward_pin, OUTPUT);
    pinMode(robot_config::motor_L_forward_pin, OUTPUT);
    pinMode(robot_config::motor_L_backward_pin, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttachChannel(robot_config::motor_R_forward_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_R_forward_pwm_channel)) {
        return false;
    }
    if (!ledcAttachChannel(robot_config::motor_R_backward_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_R_backward_pwm_channel)) {
        return false;
    }
    if (!ledcAttachChannel(robot_config::motor_L_forward_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_L_forward_pwm_channel)) {
        return false;
    }
    if (!ledcAttachChannel(robot_config::motor_L_backward_pin,
                           robot_config::motor_pwm_frequency_hz,
                           robot_config::motor_pwm_resolution_bits,
                           robot_config::motor_L_backward_pwm_channel)) {
        return false;
    }
#else
    ledcSetup(robot_config::motor_R_forward_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);
    ledcSetup(robot_config::motor_R_backward_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);
    ledcSetup(robot_config::motor_L_forward_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);
    ledcSetup(robot_config::motor_L_backward_pwm_channel,
              robot_config::motor_pwm_frequency_hz,
              robot_config::motor_pwm_resolution_bits);

    ledcAttachPin(robot_config::motor_R_forward_pin, robot_config::motor_R_forward_pwm_channel);
    ledcAttachPin(robot_config::motor_R_backward_pin, robot_config::motor_R_backward_pwm_channel);
    ledcAttachPin(robot_config::motor_L_forward_pin, robot_config::motor_L_forward_pwm_channel);
    ledcAttachPin(robot_config::motor_L_backward_pin, robot_config::motor_L_backward_pwm_channel);
#endif

    stop();
    return true;
}

void MotorDriver::writeDuty(int enable_pin, int pwm_channel, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)pwm_channel;
    ledcWrite(enable_pin, duty);
#else
    (void)enable_pin;
    ledcWrite(pwm_channel, duty);
#endif
}

void MotorDriver::writeMotor(int forward_pin,
                             int backward_pin,
                             int forward_pwm_channel,
                             int backward_pwm_channel,
                             float pwm) {
    // Meme securite qu'avant : on limite la commande a la plage autorisee
    pwm = std::fmax(-static_cast<float>(robot_config::PWM_max),
                    std::fmin(static_cast<float>(robot_config::PWM_max), pwm));

    // Meme logique de zone morte qu'avant
    if (std::fabs(pwm) < robot_config:: PWM_offset_margin) {
        pwm = 0.0f;
    }

    uint32_t duty = static_cast<uint32_t>(std::lround(std::fabs(pwm)));

    if (pwm > 0.0f) {
        // Avant : PWM sur forward, backward reste a 0
        writeDuty(forward_pin, forward_pwm_channel, duty);
        writeDuty(backward_pin, backward_pwm_channel, 0);
    } else if (pwm < 0.0f) {
        // Arriere : PWM sur backward, forward reste a 0
        writeDuty(forward_pin, forward_pwm_channel, 0);
        writeDuty(backward_pin, backward_pwm_channel, duty);
    } else {
        // Arret complet : les deux a 0
        writeDuty(forward_pin, forward_pwm_channel, 0);
        writeDuty(backward_pin, backward_pwm_channel, 0);
    }
}

void MotorDriver::write(float pwm_R, float pwm_L) {
    writeMotor(robot_config::motor_R_forward_pin,
               robot_config::motor_R_backward_pin,
               robot_config::motor_R_forward_pwm_channel,
               robot_config::motor_R_backward_pwm_channel,
               pwm_R);

    writeMotor(robot_config::motor_L_forward_pin,
               robot_config::motor_L_backward_pin,
               robot_config::motor_L_forward_pwm_channel,
               robot_config::motor_L_backward_pwm_channel,
               pwm_L);

}

void MotorDriver::stop() {
    write(0.0f, 0.0f);
}




