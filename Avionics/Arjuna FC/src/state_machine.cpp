#include "state_machine.h"
#include "config.h"
#include "sd_logger.h"
#include "telemetry.h" 
#include "sensors.h"
#include "gps.h"
#include "physics_utils.h" // For G_EARTH
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// --- STATE VARIABLES ---
static FlightState current_state = STATE_STANDBY;
static uint32_t event_counter = 0; // "count" from python logic

// --- HISTORY VARIABLES ---
static float last_altitude = 0.0f;
static float last_pressure = 0.0f;

// --- THRESHOLDS (Samples @ 150Hz) ---
// 150 samples = 1 second
const uint32_t THRESH_BOOST      = 5;  // 0.1s
const uint32_t THRESH_MACH_HIGH  = 5;  // 0.13s
const uint32_t THRESH_MACH_LOW   = 10;  
const uint32_t THRESH_MACH_MID   = 10;  
const uint32_t THRESH_APOGEE     = 10;  
const uint32_t THRESH_MAIN       = 5;  
const uint32_t THRESH_LANDING    = 20; 

void save_state_recovery() {
    RecoveryData data;
    data.magic = RECOVERY_MAGIC;
    data.state = current_state;
    data.max_altitude = last_altitude; // Roughly current altitude

    EEPROM.put(0, data);
    EEPROM.commit(); // Force write to Flash
}

void clear_state_recovery() {
    RecoveryData data;
    data.magic = 0; // Invalidate
    data.state = STATE_STANDBY;
    data.max_altitude = 0.0f;
    
    EEPROM.put(0, data);
    EEPROM.commit();
}

void attempt_recovery() {
    RecoveryData data;
    EEPROM.get(0, data);

    // 1. Check if data is valid
    if (data.magic == RECOVERY_MAGIC) {
        
        // 2. Sanity Check: Don't recover 'STANDBY' or 'RECOVERY'
        if (data.state != STATE_STANDBY && data.state != STATE_RECOVERY) {
            
            // 3. OPTIONAL: Sanity check altitude 
            // If we think we are in BOOST but altitude is 0, maybe we just reset on pad?
            // For safety, we typically TRUST the state if it's > STATE_STANDBY.
            
            current_state = data.state;
            sd_log_event("SYS_RECOVERED");
            
            // Re-engage flight power modes
            if (current_state >= STATE_BOOST) {
                exit_standby_low_power();
            }
            
            // NOTE: If we recovered into DROGUE or MAIN, 
            // the main loop will re-fire pyros if conditions met.
        }
    }
}

void state_machine_init() {
    pinMode(PYRO_DROGUE_PIN, OUTPUT);
    pinMode(PYRO_MAIN_PIN, OUTPUT);
    digitalWrite(PYRO_DROGUE_PIN, LOW);
    digitalWrite(PYRO_MAIN_PIN, LOW);

    // Initialize EEPROM (RP2040 uses Flash simulation)
    EEPROM.begin(512);
    
    current_state = STATE_STANDBY;
    event_counter = 0;
    last_altitude = 0.0f;
    last_pressure = 101325.0f;

    attempt_recovery();
}

void fire_pyro(int pin, const char* name) {
    digitalWrite(pin, HIGH);
    sd_log_event(name);
}

void enter_standby_low_power() { telemetry_set_sleep(); }
void exit_standby_low_power() { telemetry_set_normal(); }

// Helper to transition state and save it
void change_state(FlightState new_state, const char* log_msg) {
    current_state = new_state;
    event_counter = 0;
    if (log_msg) sd_log_event(log_msg);
    
    // CRITICAL: Save new state to EEPROM
    save_state_recovery();
}

void state_machine_update(RocketState* state) {
    
    // Extract current values
    float current_acc = state->acc_z_earth_frame; 
    float current_mach = state->mach_number; // Now standardized
    float current_alt = state->altitude;
    float current_pres = state->pressure_pa;

    switch (current_state) {
        
        // 1. STANDBY -> BOOST
        case STATE_STANDBY:
            // "if a_smooth > 2*g"
            if (current_acc > (2.0f * Physics::G_EARTH)) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_BOOST) {
                current_state = STATE_BOOST;
                event_counter = 0;
                exit_standby_low_power();
                sd_log_event("LAUNCH");
            }
            break;

        // 2. BOOST -> COAST_HIGH_MACH
        case STATE_BOOST:
            // "if a_smooth < 0" (Burnout)
            if (current_acc < 0.0f) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_MACH_HIGH) {
                current_state = STATE_COAST_HIGH_MACH;
                event_counter = 0;
                sd_log_event("MECO");
            }
            break;

        // 3. COAST_HIGH -> COAST_MID
        case STATE_COAST_HIGH_MACH:
            // "if mach < 0.65"
            if (current_mach < 0.7f) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_MACH_MID) {
                current_state = STATE_COAST_MID_MACH;
                event_counter = 0;
            }
            break;

        // 4. COAST_MID -> COAST_LOW
        case STATE_COAST_MID_MACH:
            // "if mach < 0.3"
            if (current_mach < 0.30f) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_MACH_LOW) {
                current_state = STATE_COAST_LOW_MACH;
                event_counter = 0;
            }
            break;

        // 5. COAST_LOW -> DROGUE (APOGEE)
        case STATE_COAST_LOW_MACH:
            // "if h_smooth[i-1] > h_smooth[i]" (Apogee)
            if (last_altitude > current_alt) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_APOGEE) {
                current_state = STATE_DROGUE;
                event_counter = 0;
                fire_pyro(PYRO_DROGUE_PIN, "APOGEE");
            }
            break;

        // 6. DROGUE -> MAIN
        case STATE_DROGUE:
            // "if h_smooth < 457" (Main Deploy)
            if (current_alt < MAIN_DEPLOY_ALT_M) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_MAIN) {
                current_state = STATE_MAIN;
                event_counter = 0;
                fire_pyro(PYRO_MAIN_PIN, "MAIN");
            }
            break;

        // 7. MAIN -> RECOVERY
        case STATE_MAIN:
            // "if abs(p_col[i] - p_col[i-1]) < 0.5" (Pressure Stable)
            if (fabsf(current_pres - last_pressure) < 0.5f) {
                event_counter++;
            } else {
                event_counter = 0;
            }

            if (event_counter >= THRESH_LANDING) {
                current_state = STATE_RECOVERY;
                event_counter = 0;
                sd_log_event("LANDED");
                enter_standby_low_power();
            }
            break;

        case STATE_RECOVERY:
            break;
    }

    last_altitude = current_alt;
    last_pressure = current_pres;
}

FlightState get_current_state() {
    return current_state;
}

const char* get_state_name(FlightState state) {
    switch(state) {
        case STATE_STANDBY:         return "STANDBY";
        case STATE_BOOST:           return "BOOST";
        case STATE_COAST_HIGH_MACH: return "COAST_HI";
        case STATE_COAST_MID_MACH:  return "COAST_MID";
        case STATE_COAST_LOW_MACH:  return "COAST_LO";
        case STATE_DROGUE:          return "DROGUE";
        case STATE_MAIN:            return "MAIN";
        case STATE_RECOVERY:        return "RECOVERY";
        default: return "UNKNOWN";
    }
}