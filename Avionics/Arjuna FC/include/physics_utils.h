#pragma once
#include <math.h>
#include <stdint.h>

namespace Physics {
    // --- Constants ---
    constexpr float G_EARTH = 9.80665f;
    constexpr float R_GAS   = 287.05f;      
    constexpr float LAPSE_L = 0.0065f;      
    constexpr float GAMMA   = 1.4f;         
    constexpr float P0_STD  = 101325.0f;    

    // --- Conversions ---
    inline float ms2_to_g(float ms2) { return ms2 / G_EARTH; }
    inline float celsius_to_kelvin(float c) { return c + 273.15f; }
    inline float deg_to_rad(float d) { return d * 0.01745329251f; }

    // --- Altitude (ISA Model) ---
    inline float altitude_from_pressure(float pressure_pa, float ground_p_pa, float ground_t_k) {
        if (ground_p_pa < 1.0f) ground_p_pa = P0_STD; 
        if (pressure_pa < 1.0f) return 0.0f; 

        const float exponent = (R_GAS * LAPSE_L) / G_EARTH; 
        float base = pressure_pa / ground_p_pa;
        float term = 1.0f - powf(base, exponent);
        
        return (ground_t_k / LAPSE_L) * term;
    }

    // --- Mach Number (Velocity Based) ---
    // Universal helper for V / A
    inline float calculate_mach_from_velocity(float velocity_mps, float speed_of_sound_mps) {
        if (speed_of_sound_mps <= 0.1f) return 0.0f;
        return fabsf(velocity_mps) / speed_of_sound_mps;
    }

    // --- Mach Number (Pressure Based / Pitot) ---
    // Saint-Venant Equation
    inline float calculate_mach_pressure(float diff_pressure_pa, float static_pressure_pa) {
        float ratio = (diff_pressure_pa / static_pressure_pa) + 1.0f;
        float exponent = (GAMMA - 1.0f) / GAMMA;
        
        float term = powf(ratio, exponent) - 1.0f;
        float factor = 2.0f / (GAMMA - 1.0f);

        float mach_sq = factor * term;
        return (mach_sq > 0.0f) ? sqrtf(mach_sq) : 0.0f;
    }

    // --- True Airspeed (TAS) ---
    inline float calculate_tas(float mach, float speed_of_sound) {
        return mach * speed_of_sound;
    }
}