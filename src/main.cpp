#include <Arduino.h>
#include "encoder_pcnt.hpp"
#include "motor_driver.hpp"
#include "diffnav.hpp"
#include "robot_config.hpp"

// =====================================================================
//                  TEST D'UN goTo (reglage)
// =====================================================================
// Le robot part de (0, 0) oriente vers +X (phi = 0).
// goTo se deroule en 3 phases :
//   ALIGN        : rotation sur place vers la cible
//   DRIVE_LINE   : ligne droite avec correction de cap et d'ecart lateral
//   FINAL_ORIENT : rotation finale (seulement si USE_FINAL_PHI = true)

// ---------- Le deplacement teste ----------
const float TARGET_X_MM      = 500.0f;
const float TARGET_Y_MM      = 300.0f;
const float CRUISE_SPEED_MM_S = 300.0f;  // vitesse de croisiere en ligne droite
const int   DIRECTION        = 1;        // 1 = marche avant, -1 = marche arriere
const bool  USE_FINAL_PHI    = true;     // orientation finale imposee ?
const float FINAL_PHI_DEG    = 0.0f;     // cap final absolu (si USE_FINAL_PHI)
const unsigned long TIMEOUT_MS = 15000;

// ---------- 1) Boucle de vitesse des roues (PI) ----------
const float KP_R = 10.0f;
const float KI_R = 50.0f;
const float KP_L = 10.0f;
const float KI_L = 50.0f;

const float R_FORWARD_MIN  =  370.0f;
const float R_BACKWARD_MIN = -370.0f;
const float L_FORWARD_MIN  =  340.0f;
const float L_BACKWARD_MIN = -340.0f;

// ---------- 2) Calibration geometrique (deja reglee avec le test de rotation) ----------
const float WHEEL_SPACING_MM = 311.2313829856f;

// ---------- 3) Rotations ALIGN et FINAL_ORIENT (valeurs du test de rotation) ----------
const float ROTATING_SPEED_MM_S = 200.0f;  // goTo utilise la vitesse de la config, pas un argument
const float ROT_ACCEL_MM_S2     = 700.0f;
const float ROT_BRAKE_MM_S2     = 600.0f;
const float ROT_MIN_SPEED_MM_S  = 35.0f;
const float FINAL_PHI_KP        = 6.0f;    // sert aussi a la toute fin de la ligne droite

// ---------- 4) Profil de la ligne droite ----------
const float LINE_ACCEL_MM_S2    = 900.0f;
const float LINE_BRAKE_MM_S2    = 1200.0f;
const float LINE_MIN_SPEED_MM_S = 55.0f;

// ---------- 5) Tenue de trajectoire (pendant la ligne droite) ----------
const float PHI_CORRECTION_KP      = 8.0f;   // correction de cap
const float LANE_GAIN_S_INV        = 12.0f;  // correction d'ecart lateral (Stanley)
const float LANE_ANGLE_GAIN        = 1.5f;
const float LANE_SOFTENING_MM_S    = 90.0f;

// ---------- 6) Approche finale (derniers 45 mm) ----------
const float FINAL_POSITION_KP_S_INV = 2.0f;   // vitesse = Kp x distance restante
const float FINAL_SPEED_LIMIT_MM_S  = 120.0f;

// ---------- 7) Tolerances d'arrivee ----------
const float POSITION_TOLERANCE_MM    = 2.0f;
const float LANE_TOLERANCE_MM        = 3.0f;
const float PHI_TOLERANCE_DEG        = 1.0f;   // fin de ligne droite et FINAL_ORIENT
const float ORIENTATION_TOLERANCE_DEG = 2.0f;  // fin de la phase ALIGN
const float STOPPED_SPEED_MM_S       = 12.0f;
const uint16_t SETTLE_CYCLES         = 8;

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

unsigned long start_ms = 0;
unsigned long phase_start_ms = 0;
diffnav::MotionMode last_mode = diffnav::MotionMode::IDLE;
float max_lane_error_mm = 0.0f;   // plus grand ecart lateral pendant la ligne droite

unsigned long last_us = 0;
unsigned long next_us = 0;
int cycle_count = 0;
bool finished = false;

const char* modeName(diffnav::MotionMode mode) {
    switch (mode) {
        case diffnav::MotionMode::IDLE:           return "IDLE";
        case diffnav::MotionMode::VELOCITY:       return "VELOCITY";
        case diffnav::MotionMode::ALIGN:          return "ALIGN";
        case diffnav::MotionMode::DRIVE_LINE:     return "DRIVE_LINE";
        case diffnav::MotionMode::ROTATE:         return "ROTATE";
        case diffnav::MotionMode::FINAL_ORIENT:   return "FINAL_ORIENT";
        case diffnav::MotionMode::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case diffnav::MotionMode::FAULT:          return "FAULT";
    }
    return "?";
}

void printReport(const char* outcome) {
    const diffnav::Pose2D& pose = odometry.state().pose;
    const float dx = TARGET_X_MM - pose.x_mm;
    const float dy = TARGET_Y_MM - pose.y_mm;

    Serial.println("----------------------------------------");
    Serial.print("RESULTAT : ");               Serial.println(outcome);
    Serial.print("  cible          : (");      Serial.print(TARGET_X_MM, 1); Serial.print(", ");
                                               Serial.print(TARGET_Y_MM, 1); Serial.println(") mm");
    Serial.print("  position finale: (");      Serial.print(pose.x_mm, 1); Serial.print(", ");
                                               Serial.print(pose.y_mm, 1); Serial.println(") mm");
    Serial.print("  erreur position: ");       Serial.print(sqrtf(dx * dx + dy * dy), 1); Serial.println(" mm");
    Serial.print("  cap final      : ");       Serial.print(pose.phi_rad * DEG_PER_RAD, 2); Serial.println(" deg");
    if (USE_FINAL_PHI) {
        Serial.print("  erreur de cap  : ");
        Serial.print(diffnav::wrapAngle(FINAL_PHI_DEG * RAD_PER_DEG - pose.phi_rad) * DEG_PER_RAD, 2);
        Serial.println(" deg");
    }
    Serial.print("  ecart lateral max (ligne droite): ");
    Serial.print(max_lane_error_mm, 1);        Serial.println(" mm");
    Serial.print("  duree totale   : ");       Serial.print(millis() - start_ms); Serial.println(" ms");
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

    // --- Odometrie ---
    diffnav::OdometryConfig odo_cfg = robot_config::odometryConfig();
    odo_cfg.wheel_spacing_mm = WHEEL_SPACING_MM;
    odometry = diffnav::DifferentialOdometry(odo_cfg);

    // --- Navigator ---
    diffnav::NavigatorConfig nav_cfg = robot_config::navigatorConfig();
    nav_cfg.wheel_spacing_mm = WHEEL_SPACING_MM;

    nav_cfg.rotating_speed_mm_s = ROTATING_SPEED_MM_S;
    nav_cfg.rotating_ramping_acceleration_mm_s2 = ROT_ACCEL_MM_S2;
    nav_cfg.rotating_breaking_acceleration_mm_s2 = ROT_BRAKE_MM_S2;
    nav_cfg.rotating_minimum_speed_mm_s = ROT_MIN_SPEED_MM_S;
    nav_cfg.final_phi_kp = FINAL_PHI_KP;

    nav_cfg.ramping_acceleration_mm_s2 = LINE_ACCEL_MM_S2;
    nav_cfg.breaking_acceleration_mm_s2 = LINE_BRAKE_MM_S2;
    nav_cfg.minimum_cruise_speed_mm_s = LINE_MIN_SPEED_MM_S;

    nav_cfg.phi_correction_kp = PHI_CORRECTION_KP;
    nav_cfg.lane_correction_gain_s_inv = LANE_GAIN_S_INV;
    nav_cfg.lane_correction_angle_gain = LANE_ANGLE_GAIN;
    nav_cfg.lane_correction_softening_mm_s = LANE_SOFTENING_MM_S;

    nav_cfg.final_position_kp_s_inv = FINAL_POSITION_KP_S_INV;
    nav_cfg.final_speed_limit_mm_s = FINAL_SPEED_LIMIT_MM_S;

    nav_cfg.position_tolerance_mm = POSITION_TOLERANCE_MM;
    nav_cfg.lane_tolerance_mm = LANE_TOLERANCE_MM;
    nav_cfg.phi_tolerance_rad = PHI_TOLERANCE_DEG * RAD_PER_DEG;
    nav_cfg.orientation_tolerance_rad = ORIENTATION_TOLERANCE_DEG * RAD_PER_DEG;
    nav_cfg.stopped_speed_tolerance_mm_s = STOPPED_SPEED_MM_S;
    nav_cfg.settle_cycles = SETTLE_CYCLES;
    navigator = diffnav::Navigator(nav_cfg);

    delay(3000);   // le temps de connecter Teleplot et de poser le robot

    EncoderCounts c = encoders.snapshot();
    diffnav::Pose2D start_pose{};
    odometry.reset(start_pose, c.encoder_count_R, c.encoder_count_L);

    navigator.goTo(start_pose, TARGET_X_MM, TARGET_Y_MM, CRUISE_SPEED_MM_S,
                   DIRECTION, USE_FINAL_PHI, FINAL_PHI_DEG * RAD_PER_DEG);

    Serial.print("DEBUT goTo vers (");
    Serial.print(TARGET_X_MM); Serial.print(", "); Serial.print(TARGET_Y_MM);
    Serial.println(")");

    start_ms = millis();
    phase_start_ms = start_ms;
    last_mode = navigator.mode();
    Serial.print("  phase "); Serial.println(modeName(last_mode));

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

    const diffnav::MotionStatus& st = navigator.status();
    const diffnav::Pose2D& pose = odometry.state().pose;

    // --- Changement de phase : affiche la duree de la phase precedente ---
    if (st.mode != last_mode) {
        const unsigned long t = millis();
        Serial.print("  phase "); Serial.print(modeName(last_mode));
        Serial.print(" terminee en "); Serial.print(t - phase_start_ms); Serial.print(" ms");
        Serial.print(" -> phase "); Serial.println(modeName(st.mode));
        last_mode = st.mode;
        phase_start_ms = t;
    }

    if (st.mode == diffnav::MotionMode::DRIVE_LINE &&
        fabsf(st.lane_error_mm) > max_lane_error_mm) {
        max_lane_error_mm = fabsf(st.lane_error_mm);
    }

    // --- Traces Teleplot ---
    if (++cycle_count % PLOT_EVERY == 0) {
        // Trajectoire dans le plan (graphique XY dans Teleplot)
        Serial.print(">trajectoire:"); Serial.print(pose.x_mm); Serial.print(":");
        Serial.print(pose.y_mm);       Serial.println("|xy");

        Serial.print(">phase:");        Serial.println(static_cast<int>(st.mode));
        Serial.print(">restant_mm:");   Serial.println(st.remaining_mm);
        Serial.print(">ecart_lat_mm:"); Serial.println(st.lane_error_mm);
        Serial.print(">err_cap_deg:");  Serial.println(st.phi_error_rad * DEG_PER_RAD);
        Serial.print(">consigne_R:");   Serial.println(target_speeds.speed_R_mm_s);
        Serial.print(">vitesse_R:");    Serial.println(speed_R);
        Serial.print(">consigne_L:");   Serial.println(target_speeds.speed_L_mm_s);
        Serial.print(">vitesse_L:");    Serial.println(speed_L);
    }

    // --- Fin du test ---
    const char* outcome = nullptr;

    if (st.result == diffnav::MotionResult::SUCCEEDED) {
        outcome = "OK";
    } else if (st.result == diffnav::MotionResult::FAULTED ||
               st.result == diffnav::MotionResult::CANCELLED) {
        outcome = "ECHEC (faute ou annulation)";
    } else if (millis() - start_ms > TIMEOUT_MS) {
        Serial.print("  bloque en phase "); Serial.println(modeName(st.mode));
        navigator.stop();
        outcome = "TIMEOUT";
    }

    if (outcome != nullptr) {
        motors.stop();
        finished = true;
        printReport(outcome);
    }
}