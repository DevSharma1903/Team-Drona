#pragma once

#include <Servo.h>
#include "control_system.h"
#include "orientation.h"
#include "flight_data.h"
#include "physics_utils.h"

class FlightCore {
public:
    // --- Subsystems ---
    ControlSystem sys;
    Orientation attitude;
    Servo airbrakeServo;

    // --- Environment State ---
    float ground_pressure_pa = 101325.0f;
    float ground_temp_k = 310.15f; // Default 37C, will calibrate

    // --- Computed Flight Metrics ---
    float speed_of_sound = 340.0f;
    float mach_number = 0.0f;
    float airspeed_tas = 0.0f; // True Airspeed (m/s)

    // --- Methods ---
    void begin(int servo_pin);
    void calibrateGroundPressure(int samples);

    // The Main Loop (Called at 150Hz from ISR)
    RocketState update(float dt, 
                       float ax, float ay, float az, 
                       float gx, float gy, float gz,
                       float mx, float my, float mz,
                       float pressure_pa, float temp_c,
                       float pitot_pa,
                       const GpsState& gps_now);
};