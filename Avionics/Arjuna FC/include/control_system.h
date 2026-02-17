#pragma once
#include "ekf/EKF.hpp"
#include "airbrakes/controller.hpp"

class ControlSystem {
public:
    // --- Subsystems ---
    rocket::EKF3 ekf;
    ab::AirbrakesController* controller = nullptr;

    // --- State Variables (Public for Telemetry) ---
    float estimated_h = 0.0f;
    float estimated_v = 0.0f;
    float estimated_az = 0.0f; 
    float servo_cmd = 0.0f;

    // --- Functions ---
    void begin();
    void update(float acc_z, float baro_alt, float dt, int state_id);
};