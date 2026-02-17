#pragma once
#include "plant.hpp"

namespace ab {

struct PredictorParams {
  float dt = 0.0067f;   
  int   max_steps = 2000;   // dt*steps = max simulated time
};

// Optional debug payload (no dynamic allocation)
struct PredictorDebug {
  float mach = 0.0f;
  float F_drag_N = 0.0f;     // signed
  float Dm_wo = 0.0f;        // signed accel contribution from base drag
  float Dm_only = 0.0f;      // signed accel contribution from delta drag at full extension
  float a_total = 0.0f;      // total accel used in integration
};

struct ApogeePredictor {
  PlantModel plant;
  PredictorParams pp;

  ApogeePredictor(const PlantParams& plant_params, const PredictorParams& pred_params)
  : plant(plant_params), pp(pred_params) {}

  // Same signature as your current version
  float predict_apogee_m(const State& now, float brake01) const {
    State x = now;
    float h_max = x.h_m;

    for (int i = 0; i < pp.max_steps; ++i) {
      if (x.v_mps <= 0.0f) break;

      plant.step(x, brake01, pp.dt);
      if (x.h_m > h_max) h_max = x.h_m;

      if (x.h_m < -1000.0f || x.h_m > 200000.0f) break;
    }
    return h_max;
  }

  // Variant that also returns the last-step debug numbers (for telemetry)
  float predict_apogee_with_debug(const State& now, float brake01, PredictorDebug& dbg) const {
    State x = now;
    float h_max = x.h_m;

    // We'll update dbg using the same underlying LUT each step,
    // but only keep the *last* computed values (cheap & deterministic).
    dbg = PredictorDebug{};

    for (int i = 0; i < pp.max_steps; ++i) {
      if (x.v_mps <= 0.0f) break;

      // --- replicate what PlantModel::accel() does, but keep intermediate debug ---
      float rho, a_sound;
      Atmosphere::isa(x.h_m, rho, a_sound);

      float mach = 0.0f, F_drag = 0.0f, Dm_wo = 0.0f, Dm_only = 0.0f;
      DragLUT::drag_subsystem(
        x.v_mps,
        brake01,
        plant.p.mass_kg,
        a_sound,
        rho,
        mach,
        F_drag,
        Dm_wo,
        Dm_only
      );

      const float inv_m = (plant.p.mass_kg > 1e-6f) ? (1.0f / plant.p.mass_kg) : 0.0f;
      const float a_drag = F_drag * inv_m;
      const float a_total = (-plant.p.g + a_drag);

      // Fill debug (last step)
      dbg.mach = mach;
      dbg.F_drag_N = F_drag;
      dbg.Dm_wo = Dm_wo;
      dbg.Dm_only = Dm_only;
      dbg.a_total = a_total;

      // --- Euler integration (same as plant.step but using our computed a_total) ---
      x.h_m   += x.v_mps * pp.dt;
      x.v_mps += a_total * pp.dt;

      if (x.h_m > h_max) h_max = x.h_m;
      if (x.h_m < -1000.0f || x.h_m > 200000.0f) break;
    }

    return h_max;
  }
};

} // namespace ab
