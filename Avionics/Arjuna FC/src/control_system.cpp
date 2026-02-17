#include "control_system.h"
#include "physics_utils.h" 
#include "airbrakes/atmosphere.hpp"
#include "airbrakes/aero_lut.hpp"
#include <Arduino.h>

// Rocket Constants
static const float MASS_KG = 33.64f;    // Mass after burnout    
static const float TARGET_APOGEE = 3048.0f; 

// Utility function for Control configuration
void ControlSystem::begin() {
    ekf.init();
    ekf.setInitialState(0.0f, 0.0f, 0.0f); 
    // Link EKF to Aero LUT
    ekf.setLUT(ab::DragLUT::DRAG_BP, ab::DragLUT::CD_TABLE, ab::DragLUT::N);

    // Controller Config
    static ab::ControllerConfig cfg{};
    cfg.target_apogee_m = TARGET_APOGEE;
    cfg.k_trim = 0.30f; // Gain for trim
    cfg.trim_limit = 0.30f; 
    cfg.use_lagged_u_for_drag = true;

    static ab::PlantParams plant{};
    plant.mass_kg = MASS_KG;
    plant.g = Physics::G_EARTH; 

    static ab::PredictorParams pred{};
    pred.dt = 0.02f;
    pred.max_steps = 2500;

    static ab::Hysteresis hyst{};
    hyst.on_threshold_m = 15.0f;
    hyst.off_threshold_m = 5.0f;

    static ab::ActuatorParams act{}; //need to map this to actual servo params
    act.tau_s = 0.10f; 
    act.min_cmd = 0.0f;
    act.max_cmd = 1.0f;

    static ab::ModelParams mp{};
    mp.v_eps = 2.5f; // deadband for velocity
    mp.u_min = 0.0f;
    mp.u_max = 1.0f;

    static ab::SMCReachingLawParams sp{};
    sp.C0 = 1.0f; sp.C1 = 0.6f;
    sp.eta = 0.2f; sp.alpha = 0.6f;
    sp.K = 0.1f; sp.phi = 10.0f;
    sp.v_eps = mp.v_eps;

    controller = new ab::AirbrakesController(cfg, plant, pred, hyst, act, mp, sp);
}

void ControlSystem::update(float acc_z_earth, float baro_alt, float dt, int state_id) {
    
    int ekf_mode = 0;
    if (state_id == 2) ekf_mode = 1;      // BOOST
    else if (state_id >= 3) ekf_mode = 2; // COAST

    // 2. Track time since boost for the dynamic mass model
    static uint32_t boost_start_ms = 0;
    float t_boost = 0.0f;

    if (state_id == 2) { // Just entered or in BOOST
        if (boost_start_ms == 0) boost_start_ms = millis();
        t_boost = (float)(millis() - boost_start_ms) / 1000.0f;
    } else if (state_id > 2) {
        // If we are in coast, t_boost is effectively > 4s, 
        // the EKF logic will handle the clipping to dry mass.
        t_boost = (float)(millis() - boost_start_ms) / 1000.0f;
    }
    ekf.step(ekf_mode, baro_alt, acc_z_earth, dt, t_boost);

    estimated_h = ekf.X.x0;
    estimated_v = ekf.X.x1;
    estimated_az = acc_z_earth; 
    
    // 2. Control Logic
    bool allow_control = (state_id == 3); 

    if (state_id == 3) { 
        float v_current = fabsf(estimated_v); 
        float h_current = estimated_h;
        
        // --- UNIVERSAL ATMOSPHERE & MACH ---
        float rho = 0.0f;
        float a_sound = 0.0f;
        ab::Atmosphere::isa(h_current, rho, a_sound); //

        // Consistent Mach Calculation
        float mach = Physics::calculate_mach_from_velocity(v_current, a_sound);

        ab::EstimatorInput in{};
        in.h_m = h_current;
        in.v_mps = estimated_v;
        in.rho_kgm3 = rho;
        // If EstimatorInput supports rho/mach, assign them here.
        // Otherwise silence warnings:
        (void)mach; // If you don't use it, just silence the warning.

        ab::Command out = controller->step(in, dt, allow_control);
        servo_cmd = out.u;
        /// Mapping function of u to servo angle

    } else {
        servo_cmd = 0.0f;
    }
}