#include <Arduino.h>
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "diffnav.hpp"
#include "robot_config.hpp"

// ---------- Reglages du test (a modifier entre les essais) ----------
const float TARGET_SPEED_MM_S = 100.0f;   // roue droite = +cible, roue gauche = -cible

// Gains (un jeu par roue, pour pouvoir les separer plus tard)
const float KP_R = 2.0f;
const float KI_R = 15.0f;
const float KP_L =2.0f;
const float KI_L = 15.0f;

// Zones mortes mesurees. Les valeurs "backward" sont NEGATIVES (comme dans robot_config.hpp)
const float R_FORWARD_MIN  =  370.0f;
const float R_BACKWARD_MIN = -370.0f;   // <-- a MESURER (valeur provisoire)
const float L_FORWARD_MIN  =  370.0f;
const float L_BACKWARD_MIN = -370.0f;   // <-- a MESURER (valeur provisoire)

const int PLOT_EVERY = 20;              // 1 point toutes les 20 iterations (~50 Hz)
// ---------------------------------------------------------------------

const unsigned long PERIOD_US = robot_config::control_period_us;

EncoderPcnt encoders;
MotorDriver motors;
diffnav::DifferentialOdometry odometry(robot_config::odometryConfig());
diffnav::WheelSpeedController controller_R(robot_config::wheelControllerConfig_R());
diffnav::WheelSpeedController controller_L(robot_config::wheelControllerConfig_L());

unsigned long last_us = 0;
unsigned long next_us = 0;
int cycle_count = 0;
bool finished = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    encoders.begin();
    motors.begin();

    diffnav::WheelControllerConfig cfg_R = robot_config::wheelControllerConfig_R();
    cfg_R.speed_kp = KP_R;
    cfg_R.speed_ki = KI_R;
    cfg_R.pwm_forward_min = R_FORWARD_MIN;
    cfg_R.pwm_backward_min = R_BACKWARD_MIN;
    controller_R = diffnav::WheelSpeedController(cfg_R);
    controller_R.reset();

    diffnav::WheelControllerConfig cfg_L = robot_config::wheelControllerConfig_L();
    cfg_L.speed_kp = KP_L;
    cfg_L.speed_ki = KI_L;
    cfg_L.pwm_forward_min = L_FORWARD_MIN;
    cfg_L.pwm_backward_min = L_BACKWARD_MIN;
    controller_L = diffnav::WheelSpeedController(cfg_L);
    controller_L.reset();

    delay(3000);   // le temps de connecter Teleplot et de poser le robot

    EncoderCounts c = encoders.snapshot();
    odometry.reset(diffnav::Pose2D{}, c.encoder_count_R, c.encoder_count_L);

    last_us = micros();
    next_us = last_us + PERIOD_US;
}

void loop() {
    if (finished) return;

    // Arret sur commande : la lettre 's' recue par le port serie
    // if (Serial.available() > 0) {
    //     char c = Serial.read();
    //     if (c == 's' || c == 'S') {
    //         motors.stop();
    //         controller_R.reset();
    //         controller_L.reset();
    //         finished = true;
    //         Serial.println("FIN");
    //         return;
    //     }
    // }

    unsigned long now = micros();
    if ((long)(now - next_us) < 0) return;

    float dt_s = (now - last_us) * 1e-6f;
    last_us = now;
    next_us += PERIOD_US;
    if ((long)(now - next_us) > 0) next_us = now + PERIOD_US;

    EncoderCounts counts = encoders.snapshot();
    odometry.update(counts.encoder_count_R, counts.encoder_count_L, dt_s);

    // Vitesses des deux roues calculees par diffnav
    float speed_R = odometry.state().wheel_speed.speed_R_mm_s;
    float speed_L = odometry.state().wheel_speed.speed_L_mm_s;

    // Cibles opposees : le robot tourne sur lui-meme
    float target_R =  TARGET_SPEED_MM_S;
    float target_L = TARGET_SPEED_MM_S;

    float pwm_R = controller_R.update(target_R, speed_R, dt_s);
    float pwm_L = controller_L.update(target_L, speed_L, dt_s);
    motors.write(pwm_R, pwm_L);

    // // Format Teleplot : une ligne par courbe
     if (++cycle_count % PLOT_EVERY == 0) {
    //    Serial.print(">target_R:"); Serial.println(target_R);
        Serial.print(">speed_R:");  Serial.println(speed_R);
        Serial.print(">pwm_R:");    Serial.println(pwm_R);
    //     Serial.print(">target_L:"); Serial.println(target_L);
    //     Serial.print(">speed_L:");  Serial.println(speed_L);
    //     Serial.print(">pwm_L:");    Serial.println(pwm_L);
    }
}