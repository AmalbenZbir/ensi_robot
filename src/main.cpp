#include <Arduino.h>
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "diffnav.hpp"
#include "robot_config.hpp"

// ---------- Reglages du test ----------
const float MOVE_DISTANCE_MM = 1000.0f;   // distance a parcourir
const float CRUISE_SPEED_MM_S = 200.0f;   // vitesse de croisiere demandee

const float KP_R = 10.0f;
const float KI_R = 50.0f;
const float KP_L = 10.0f;
const float KI_L = 50.0f;

const float R_FORWARD_MIN  =  370.0f;
const float R_BACKWARD_MIN = -370.0f;
const float L_FORWARD_MIN  =  370.0f;
const float L_BACKWARD_MIN = -370.0f;

const int PLOT_EVERY = 20;
// ---------------------------------------------------------------------

const unsigned long PERIOD_US = robot_config::control_period_us;

EncoderPcnt encoders;
MotorDriver motors;
diffnav::DifferentialOdometry odometry(robot_config::odometryConfig());
diffnav::Navigator navigator(robot_config::navigatorConfig());

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
    diffnav::Pose2D start_pose{};
    odometry.reset(start_pose, c.encoder_count_R, c.encoder_count_L);

    // Demarre le mouvement une seule fois, ici, avant d'entrer dans loop()
    navigator.moveForward(start_pose, MOVE_DISTANCE_MM, CRUISE_SPEED_MM_S);

    last_us = micros();
    next_us = last_us + PERIOD_US;
}

void loop() {
    if (finished) return;

    unsigned long now = micros();
    if ((long)(now - next_us) < 0) return;

    float dt_s = (now - last_us) * 1e-6f;
    last_us = now;
    next_us += PERIOD_US;
    if ((long)(now - next_us) > 0) next_us = now + PERIOD_US;

    EncoderCounts counts = encoders.snapshot();
    odometry.update(counts.encoder_count_R, counts.encoder_count_L, dt_s);

    // Le Navigator calcule les vitesses de roues cibles a partir de l'etat actuel
    diffnav::WheelSpeeds target_speeds = navigator.update(odometry.state(), dt_s);

    float speed_R = odometry.state().wheel_speed.speed_R_mm_s;
    float speed_L = odometry.state().wheel_speed.speed_L_mm_s;

    float pwm_R = controller_R.update(target_speeds.speed_R_mm_s, speed_R, dt_s);
    float pwm_L = controller_L.update(target_speeds.speed_L_mm_s, speed_L, dt_s);
    motors.write(pwm_R, pwm_L);

    if (++cycle_count % PLOT_EVERY == 0) {
        Serial.print(">x_mm:");     Serial.println(odometry.state().pose.x_mm);
        Serial.print(">y_mm:");     Serial.println(odometry.state().pose.y_mm);
    }

    // Arret automatique une fois le mouvement termine
    if (navigator.status().result == diffnav::MotionResult::SUCCEEDED) {
        motors.stop();
        finished = true;
        Serial.println("FIN, mouvement termine");
    }
}