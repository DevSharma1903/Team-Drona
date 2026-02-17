#pragma once
#include <cmath>
#include "types.hpp"
#include "aero_lut.hpp"     // DragLUT::drag_subsystem(...)
#include "atmosphere.hpp"   // Atmosphere::isa(...)

namespace ab {

struct PlantParams {
  float mass_kg;      // current mass (coast mass)
  float g = 9.80665f; // gravity (m/s^2)
};

struct PlantModel {
  PlantParams p;

  explicit PlantModel(const PlantParams& params) : p(params) {}

  // Compute acceleration (vertical, up positive)
  float accel(const State& x, float brake01) const {
    float rho, a_sound;
    Atmosphere::isa(x.h_m, rho, a_sound);

    float mach = 0.0f;
    float F_drag = 0.0f;   // signed (N)
    float Dm_wo = 0.0f;    // signed accel component from base drag (m/s^2)
    float Dm_only = 0.0f;  // signed accel component from delta drag at full extension (m/s^2)

    DragLUT::drag_subsystem(
      x.v_mps,
      brake01,     // u_with_lag in your Python (already lagged signal)
      p.mass_kg,
      a_sound,
      rho,
      mach,
      F_drag,
      Dm_wo,
      Dm_only
    );

    // Total acceleration:
    // a = -g + (F_drag / m)
    // Note: F_drag is already signed by velocity direction exactly like Python.
    const float inv_m = (p.mass_kg > 1e-6f) ? (1.0f / p.mass_kg) : 0.0f;
    const float a_drag = F_drag * inv_m;

    return (-p.g + a_drag);
  }

  // Euler integrate state (coast phase)
  void step(State& x, float brake01, float dt) const {
    const float a = accel(x, brake01);
    x.h_m   += x.v_mps * dt;
    x.v_mps += a * dt;
  }
};

} // namespace ab
