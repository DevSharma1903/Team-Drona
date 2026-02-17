#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm> // for std::max
#include "types.hpp" // for clampf

namespace ab {

struct DragLUT {
  static constexpr int N = 23;

  // --- [CHANGED] Constants for Geometry ---
  // A0: Reference area for the airbrakes (m^2 per "u")
  static constexpr float A0 = 0.0020f; 
  
  // A_REF: Reference area for the rocket body (m^2)
  // Derived from previous LUT: F ~ 109N @ Mach 0.4 (SL) -> A_ref ~ 0.0177 m^2 (~150mm dia)
  // [USER: VERIFY THIS VALUE FOR YOUR AIRFRAME]
  static constexpr float A_REF = 0.0176f; 

  // --- Breakpoints (Kept as requested) ---
  static constexpr float DRAG_BP[N] = {
    0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35,
    0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 
    0.80, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10
  };

  // --- Cd Table (Kept for Baseline Cd0 lookup) ---
  // Replaces the raw Force table with the Coefficient of Drag
  static constexpr float CD_TABLE[N] = { 
    1.38, 0.501, 0.502, 0.504, 0.507, 0.509, 0.513,
    0.517, 0.522, 0.528, 0.534, 0.541, 0.549, 0.557,
    0.566, 0.575, 0.586, 0.597, 0.608, 0.616, 0.625, 
    0.664, 0.664
  };

  // --- [REMOVED] DRAG_TABLE and DELTA_DRAG_TABLE --- 
  // (Logic replaced by dynamic calculation below)

  // --- Helper: Linear Interpolation ---
  static inline float interp_1d_clip(float x, const float* bp, const float* table) {
    if (x <= bp[0]) return table[0];
    if (x >= bp[N - 1]) return table[N - 1];

    int i = 0;
    for (; i < N - 1; ++i) {
      if (x < bp[i + 1]) break;
    }

    const float x0 = bp[i];
    const float x1 = bp[i + 1];
    const float y0 = table[i];
    const float y1 = table[i + 1];

    const float t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  // --- [CHANGED] Get Baseline Cd0 ---
  static inline float get_cd0(float mach) {
    return interp_1d_clip(mach, DRAG_BP, CD_TABLE);
  }

  // --- [NEW] Calculate Delta Cd based on Mach and Extension (u) ---
  // Implements the polynomial from your Python snippet
  static inline float cd_delta(float mach, float u) {
    // Clamp u to [0,1]
    float u_clamped = (u < 0.0f) ? 0.0f : (u > 1.0f ? 1.0f : u);
    
    float Ma = mach;
    float Ma2 = Ma * Ma;
    float Ma3 = Ma2 * Ma;
    float u2 = u_clamped * u_clamped;
    float u3 = u2 * u_clamped;

    return (
        9.6187f  * Ma * u_clamped
      - 19.0139f * Ma2 * u_clamped
      + 1.1245f  * Ma * u2
      + 10.2736f * Ma3 * u_clamped
      + 3.7661f  * Ma2 * u2
      - 3.0457f  * Ma * u3
    );
  }

  // --- [UPDATED] Drag Subsystem Physics ---
  // Now accepts 'rho' (air density) and calculates Forces dynamically
  static inline void drag_subsystem(
      float v_mps,
      float u_with_lag,
      float mass_kg,
      float a_sound,
      float rho,       // [ADDED] Required for dynamic pressure q
      float& mach_out,
      float& F_drag_out,
      float& Dm_wo_out,
      float& Dm_only_out
  ) {
    // 1. Calculate Mach
    const float v_abs = std::fabs(v_mps);
    // Avoid division by zero
    const float a_safe = (a_sound > 1e-6f) ? a_sound : 340.0f; 
    const float mach = v_abs / a_safe;

    // 2. Dynamic Pressure q = 0.5 * rho * v^2
    const float q = 0.5f * rho * v_mps * v_mps;

    // 3. Baseline Drag (No Brakes)
    // F_wo = q * Aref * Cd0
    float Cd0 = get_cd0(mach);
    float F_wo_mag = q * A_REF * Cd0;

    // 4. Delta Drag from Brakes
    // dCd = poly(Ma, u)
    // F_delta = q * (A0 * u) * dCd  <-- Note: Formula uses Area scaling A0*u
    float u = (u_with_lag < 0.0f) ? 0.0f : (u_with_lag > 1.0f ? 1.0f : u_with_lag);
    float dCd = cd_delta(mach, u);
    float A_u = A0 * u;
    
    // Note: The python snippet implies dCd is a coefficient multiplier for the Area A_u
    // F_delta_mag = q * A_u * dCd
    float F_delta_mag = 2 * q * A_u * dCd;

    // 5. Total Force Magnitude
    float F_mag = F_wo_mag + F_delta_mag;

    // 6. Apply Sign (Drag opposes velocity)
    float F_drag = 0.0f;
    float sgn = 0.0f;

    if (v_mps > 0.0f) {
      F_drag = -F_mag;
      sgn = 1.0f;
    } else if (v_mps < 0.0f) {
      F_drag = +F_mag;
      sgn = -1.0f;
    } else {
      F_drag = 0.0f;
      sgn = 0.0f;
    }

    // 7. Calculate Decelerations (F=ma -> a=F/m)
    const float inv_m = (mass_kg > 1e-6f) ? (1.0f / mass_kg) : 0.0f;
    
    float Dm_wo   = -(sgn * F_wo_mag) * inv_m;
    float Dm_only = -(sgn * F_delta_mag) * inv_m;

    // 8. Output
    mach_out = mach;
    F_drag_out = F_drag;
    Dm_wo_out = Dm_wo;
    Dm_only_out = Dm_only;
  }
};

// Out-of-class definitions
constexpr float DragLUT::DRAG_BP[DragLUT::N];
constexpr float DragLUT::CD_TABLE[DragLUT::N];
// Removed DRAG_TABLE / DELTA_DRAG_TABLE definitions

} // namespace ab