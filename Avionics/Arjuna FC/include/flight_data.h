#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <stdint.h>

// =========================================================
// FLIGHT STATES (RESTORED ORIGINAL)
// =========================================================
typedef enum {
    STATE_STANDBY = 0,         // On pad
    STATE_BOOST = 1,           // Motor burning
    STATE_COAST_HIGH_MACH = 2, // Mach > 0.8
    STATE_COAST_MID_MACH = 3,  // 0.4 < Mach < 0.8 (Airbrakes?)
    STATE_COAST_LOW_MACH = 4,  // Mach < 0.4
    STATE_DROGUE = 5,          // Apogee
    STATE_MAIN = 6,            // Main Chute
    STATE_RECOVERY = 7         // Landed
} FlightState;

// =========================================================
// UNIFIED ROCKET STATE (150 Hz)
// =========================================================
struct __attribute__((packed)) RocketState {
    uint32_t timestamp_ms;
    uint8_t  fsm_state;

    // --- Estimator State ---
    float altitude;          
    float velocity;          
    float acc_z_earth_frame; 

    // --- Control System Data ---
    float airbrake_extension;

    // --- Physics Data ---
    float mach_number;       
    float airspeed_tas;      
    float tilt_angle_deg;    

    // --- Raw Sensors ---
    float ax_body, ay_body, az_body; 
    float gx_deg, gy_deg, gz_deg;    
    float mx_ut, my_ut, mz_ut;       

    // --- Environment ---
    float pressure_pa;
    float temp_c;
    float diff_pressure_pa; 

    // --- CONTINUITY STATUS (NEW) ---
    uint8_t drogue_continuity; // 1 = OK, 0 = OPEN
    uint8_t main_continuity;   // 1 = OK, 0 = OPEN

    // --- GPS ---
    int32_t  gps_lat;
    int32_t  gps_lon;
    float    gps_alt;
    uint32_t gps_iTOW;
    uint8_t  gps_sats;
    uint8_t  gps_fix;
};
// =========================================================
// GPS SNAPSHOT (Shared Atomic)
// =========================================================
struct __attribute__((packed)) GpsState {
    uint32_t iTOW;          // GPS Time of Week
    int32_t  lat;           // Latitude (deg * 10^7)
    int32_t  lon;           // Longitude (deg * 10^7)
    int32_t  alt_msl;       // Altitude MSL (mm)
    uint8_t  fix_type;      // 3 = 3D Fix
    uint8_t  num_sats;
    bool     updated;       // New data flag
};

#endif // FLIGHT_DATA_H