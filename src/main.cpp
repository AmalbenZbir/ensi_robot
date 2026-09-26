#include <Arduino.h>
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "diffnav.hpp"
#include "robot_config.hpp"

EncoderPcnt encoders;
MotorDriver motors;
diffnav::DifferentialOdometry odometry(robot_config::odometryConfig());

diffnav::WheelControllerConfig config;
diffnav::WheelSpeedController controller(config);

const float TARGET_SPEED_MM_S = 100.0f;

unsigned long previous_time_us = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    encoders.begin();
    motors.begin();

    EncoderCounts counts = encoders.snapshot();
    diffnav::Pose2D initial_pose{};
    odometry.reset(initial_pose, counts.encoder_count_R, counts.encoder_count_L);

    previous_time_us = micros();

    // Reglages a ajuster ici pour tes essais
    config.static_feedforward_pwm = 370.0f;
    config.speed_kp = 1.0f;
    config.speed_ki = 1.0f;
    controller = diffnav::WheelSpeedController(config);

    delay(1000);
    Serial.println("Test AVEC PID, vitesse lue depuis DifferentialOdometry.");
}

void loop() {
    unsigned long now_us = micros();
    float dt_s = (now_us - previous_time_us) / 1000000.0f;
    previous_time_us = now_us;

    if (dt_s <= 0.0f) {
        return;
    }

    EncoderCounts counts = encoders.snapshot();
    odometry.update(counts.encoder_count_R, counts.encoder_count_L, dt_s);

    float measured_speed_R_mm_s = odometry.state().wheel_speed.speed_R_mm_s;

    // PID : compare et corrige
    float pwm_R = controller.update(TARGET_SPEED_MM_S, measured_speed_R_mm_s, dt_s);

    motors.write(pwm_R, 0.0f);

    Serial.print(">target:");
    Serial.println(TARGET_SPEED_MM_S);

    Serial.print(">measured:");
    Serial.println(measured_speed_R_mm_s);

    delay(10);
}