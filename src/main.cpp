#include <Arduino.h>
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "diffnav.hpp"
#include "robot_config.hpp"

// =====================================================================
//                  TEST D'UNE SEULE ROTATION (reglage)
// =====================================================================

// ---------- La rotation testee ----------
const float ROTATION_ANGLE_DEG  = -20.0f;   // positif = antihoraire, negatif = horaire
const float ROTATING_SPEED_MM_S = 200.0f;  // vitesse de croisiere des roues (mm/s)
const unsigned long TIMEOUT_MS  = 8000;    // abandon si la rotation ne se termine pas

// ---------- 1) Boucle de vitesse des roues (PI) : a regler EN PREMIER ----------
const float KP_R = 10.0f;
const float KI_R = 50.0f;
const float KP_L = 10.0f;
const float KI_L = 50.0f;

// PWM minimal qui fait demarrer chaque moteur (zone morte)
const float R_FORWARD_MIN  =  370.0f;
const float R_BACKWARD_MIN = -370.0f;
const float L_FORWARD_MIN  =  340.0f;   // robot_config utilise 340 pour la roue gauche
const float L_BACKWARD_MIN = -340.0f;

// ---------- 2) Calibration geometrique ----------
// Entraxe des roues : determine la precision de l'angle mesure.
// Modifie A LA FOIS l'odometrie et le Navigator.
const float WHEEL_SPACING_MM = 308.0f;

// ---------- 3) Profil de rotation (Navigator) ----------
const float ROT_ACCEL_MM_S2     = 700.0f;  // acceleration au demarrage
const float ROT_BRAKE_MM_S2     = 900.0f;  // deceleration avant la cible
const float ROT_MIN_SPEED_MM_S  = 35.0f;   // vitesse mini du profil

// ---------- 4) Fin de rotation (Navigator) ----------
const float FINAL_PHI_KP        = 6.0f;    // gain P dans les derniers ~2 degres
const float PHI_TOLERANCE_DEG   = 1.0f;    // tolerance d'arrivee
const float STOPPED_SPEED_MM_S  = 12.0f;   // seuil "robot arrete"
const uint16_t SETTLE_CYCLES    = 8;       // cycles consecutifs dans la tolerance

const int PLOT_EVERY = 10;   // une trace toutes les 10 ms
// =====================================================================

const float RAD_PER_DEG = diffnav::kPi / 180.0f;
const float DEG_PER_RAD = 180.0f / diffnav::kPi;

const unsigned long PERIOD_US = robot_config::control_period_us;

EncoderPcnt encoders;
MotorDriver motors;
diffnav::DifferentialOdometry odometry(robot_config::odometryConfig());
diffnav::Navigator navigator(robot_config::navigatorConfig());

diffnav::WheelSpeedController controller_R(robot_config::wheelControllerConfig_R());
diffnav::WheelSpeedController controller_L(robot_config::wheelControllerConfig_L());

float target_absolute_phi_rad = 0.0f;
float max_overshoot_deg = 0.0f;       // plus grand depassement de la cible
unsigned long start_ms = 0;
unsigned long first_in_tolerance_ms = 0;

unsigned long last_us = 0;
unsigned long next_us = 0;
int cycle_count = 0;
bool finished = false;

void printReport(const char* outcome) {
    const float phi_now = odometry.state().pose.absolute_phi_rad;
    Serial.println("----------------------------------------");
    Serial.print("RESULTAT : ");                 Serial.println(outcome);
    Serial.print("  angle demande  : ");         Serial.print(ROTATION_ANGLE_DEG, 2); Serial.println(" deg");
    Serial.print("  angle mesure   : ");         Serial.print(phi_now * DEG_PER_RAD, 2); Serial.println(" deg");
    Serial.print("  erreur finale  : ");
    Serial.print((target_absolute_phi_rad - phi_now) * DEG_PER_RAD, 2); Serial.println(" deg");
    Serial.print("  depassement max: ");         Serial.print(max_overshoot_deg, 2); Serial.println(" deg");
    Serial.print("  duree totale   : ");         Serial.print(millis() - start_ms); Serial.println(" ms");
    if (first_in_tolerance_ms > 0) {
        Serial.print("  1ere entree dans la tolerance : ");
        Serial.print(first_in_tolerance_ms - start_ms); Serial.println(" ms");
    }
    Serial.print("  derive x/y     : ");
    Serial.print(odometry.state().pose.x_mm, 1); Serial.print(" / ");
    Serial.print(odometry.state().pose.y_mm, 1); Serial.println(" mm");
    Serial.println("----------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    encoders.begin();
    motors.begin();

    // --- Correcteurs de vitesse des roues ---
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

    // --- Odometrie (entraxe) ---
    diffnav::OdometryConfig odo_cfg = robot_config::odometryConfig();
    odo_cfg.wheel_spacing_mm = WHEEL_SPACING_MM;
    odometry = diffnav::DifferentialOdometry(odo_cfg);

    // --- Navigator (profil + fin de rotation) ---
    diffnav::NavigatorConfig nav_cfg = robot_config::navigatorConfig();
    nav_cfg.wheel_spacing_mm = WHEEL_SPACING_MM;
    nav_cfg.rotating_ramping_acceleration_mm_s2 = ROT_ACCEL_MM_S2;
    nav_cfg.rotating_breaking_acceleration_mm_s2 = ROT_BRAKE_MM_S2;
    nav_cfg.rotating_minimum_speed_mm_s = ROT_MIN_SPEED_MM_S;
    nav_cfg.final_phi_kp = FINAL_PHI_KP;
    nav_cfg.phi_tolerance_rad = PHI_TOLERANCE_DEG * RAD_PER_DEG;
    nav_cfg.stopped_speed_tolerance_mm_s = STOPPED_SPEED_MM_S;
    nav_cfg.settle_cycles = SETTLE_CYCLES;
    navigator = diffnav::Navigator(nav_cfg);

    delay(3000);   // le temps de connecter Teleplot et de poser le robot

    EncoderCounts c = encoders.snapshot();
    diffnav::Pose2D start_pose{};
    odometry.reset(start_pose, c.encoder_count_R, c.encoder_count_L);

    // Lance l'unique rotation
    const float angle_rad = ROTATION_ANGLE_DEG * RAD_PER_DEG;
    target_absolute_phi_rad = start_pose.absolute_phi_rad + angle_rad;
    navigator.rotate(start_pose, angle_rad, ROTATING_SPEED_MM_S);

    Serial.print("DEBUT rotation de ");
    Serial.print(ROTATION_ANGLE_DEG);
    Serial.println(" deg");

    start_ms = millis();
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

    diffnav::WheelSpeeds target_speeds = navigator.update(odometry.state(), dt_s);

    float speed_R = odometry.state().wheel_speed.speed_R_mm_s;
    float speed_L = odometry.state().wheel_speed.speed_L_mm_s;

    float pwm_R = controller_R.update(target_speeds.speed_R_mm_s, speed_R, dt_s);
    float pwm_L = controller_L.update(target_speeds.speed_L_mm_s, speed_L, dt_s);
    motors.write(pwm_R, pwm_L);

    // --- Statistiques pour le bilan ---
    const float phi_now = odometry.state().pose.absolute_phi_rad;
    const float error_rad = target_absolute_phi_rad - phi_now;
    // Depassement = erreur de signe oppose a la rotation demandee
    const float overshoot_deg = -diffnav::sign(ROTATION_ANGLE_DEG) * error_rad * DEG_PER_RAD;
    if (overshoot_deg > max_overshoot_deg) max_overshoot_deg = overshoot_deg;
    if (first_in_tolerance_ms == 0 && fabsf(error_rad) <= PHI_TOLERANCE_DEG * RAD_PER_DEG) {
        first_in_tolerance_ms = millis();
    }

    // --- Traces Teleplot ---
    if (++cycle_count % PLOT_EVERY == 0) {
        Serial.print(">phi_deg:");     Serial.println(phi_now * DEG_PER_RAD);
        Serial.print(">erreur_deg:");  Serial.println(error_rad * DEG_PER_RAD);
        // Serial.print(">consigne_R:");  Serial.println(target_speeds.speed_R_mm_s);
        // Serial.print(">vitesse_R:");   Serial.println(speed_R);
        // Serial.print(">consigne_L:");  Serial.println(target_speeds.speed_L_mm_s);
        // Serial.print(">vitesse_L:");   Serial.println(speed_L);
        // Serial.print(">x_mm:");        Serial.println(odometry.state().pose.x_mm);
        // Serial.print(">y_mm:");        Serial.println(odometry.state().pose.y_mm);
    }

    // --- Fin du test ---
    // const diffnav::MotionResult result = navigator.status().result;
    // const char* outcome = nullptr;

    // if (result == diffnav::MotionResult::SUCCEEDED) {
    //     outcome = "OK";
    // } else if (result == diffnav::MotionResult::FAULTED ||
    //            result == diffnav::MotionResult::CANCELLED) {
    //     outcome = "ECHEC (faute ou annulation)";
    // } else if (millis() - start_ms > TIMEOUT_MS) {
    //     navigator.stop();
    //     outcome = "TIMEOUT (robot bloque hors tolerance ?)";
    // }

    // if (outcome != nullptr) {
    //     motors.stop();
    //     finished = true;
    //     printReport(outcome);
    // }
}