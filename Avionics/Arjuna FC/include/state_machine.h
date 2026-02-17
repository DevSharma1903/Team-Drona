#pragma once
#include "flight_data.h"

// Magic number to detect if EEPROM data is valid (e.g., "ARJ1")
const uint32_t RECOVERY_MAGIC = 0x41524A31; 

struct RecoveryData {
    uint32_t magic;       // Integrity check
    FlightState state;    // The state we were in
    float max_altitude;   // To prevent re-firing apogee if we already passed it
};

void state_machine_init();
void state_machine_update(RocketState* state);
FlightState get_current_state();
const char* get_state_name(FlightState state);

void exit_standby_low_power();
void enter_standby_low_power();



void save_state_recovery();
void clear_state_recovery();