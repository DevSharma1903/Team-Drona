#pragma once
#include "types.hpp"
#include "model.hpp"
#include "smc.hpp"
#include "hysteresis.hpp"
#include "actuator.hpp"
#include "predictor.hpp"
#include "command.hpp"
#include "atmosphere.hpp"
#include "aero_lut.hpp"

namespace ab {

struct ControllerConfig {
  float target_apogee_m = 3048.0f; // 10,000 ft default

  // Trim settings
  float k_trim = 0.3f;
  float trim_limit = 0.3f; 

  // use_lagged_u_for_drag=true matches Python "u_with_lag".
  bool use_lagged_u_for_drag = true;
};

struct AirbrakesController {
  ControllerConfig cfg;

  // Subsystems
  ApogeePredictor predictor;
  Hysteresis hyst;
  Actuator1stOrder actuator;

  // Params needed by command law
  ModelParams model_params;
  SMCReachingLawParams smc_params;

  // Last computed values (telemetry/debug)
  float last_pred_no = 0.0f;
  float last_pred_full = 0.0f;
  float last_error = 0.0f;
  bool  last_enabled = false;

  float last_mach = 0.0f;
  float last_F_drag = 0.0f;
  float last_Dm_wo = 0.0f;
  float last_Dm_only = 0.0f;

  float last_u_cmd = 0.0f;
  CmdDebug last_dbg{};

  AirbrakesController(
    const ControllerConfig& c,
    const PlantParams& plant_params,
    const PredictorParams& pred_params,
    const Hysteresis& hyst_params,
    const ActuatorParams& act_params,
    const ModelParams& mp,
    const SMCReachingLawParams& sp
  )
  : cfg(c),
    predictor(plant_params, pred_params),
    hyst(hyst_params),
    actuator(act_params),
    model_params(mp),
    smc_params(sp)
  {}

  // One real-time step
  Command step(const EstimatorInput& in, float dt, bool allow_authority) {
    // Current estimated state
    State now{in.h_m, in.v_mps};

    // --- (1) Predict apogee for u=0 and u=1 ---
    last_pred_no   = predictor.predict_apogee_m(now, 0.0f);
    last_pred_full = predictor.predict_apogee_m(now, 1.0f);

    // Error uses pred_no
    last_error = last_pred_no - cfg.target_apogee_m;

    // --- (2) Hysteresis gating + authority ---
    bool enabled = hyst.update(last_error);
    enabled = enabled && allow_authority;
    last_enabled = enabled;

    // --- (3) Compute Drag components using LUT ---
    // We calculate rho/a_sound locally here to ensure we have them for the drag physics
    float rho, a_sound;
    Atmosphere::isa(now.h_m, rho, a_sound);

    const float u_for_drag = cfg.use_lagged_u_for_drag ? actuator.y : 0.0f;

    // [CHANGED] Updated to match new drag_subsystem signature
    DragLUT::drag_subsystem(
      now.v_mps,
      u_for_drag,                   
      predictor.plant.p.mass_kg,    
      a_sound,
      rho,                          // <--- ADDED: Air Density
      last_mach,                    // Output: Mach
      last_F_drag,                  // Output: Total Drag Force
      last_Dm_wo,                   // Output: Decel (Base)
      last_Dm_only                  // Output: Decel (Brakes)
    );

    // --- (4) Command law ---
    ModelState st{};
    st.h = now.h_m;
    st.v = now.v_mps;
    st.s = last_dbg.s; 

    CmdDebug dbg{};
    const float u_cmd = compute_control_command(
      st,
      last_pred_no,
      last_pred_full,
      cfg.target_apogee_m,
      enabled,
      last_Dm_wo,
      last_Dm_only,
      model_params,
      smc_params,
      cfg.k_trim,
      cfg.trim_limit,
      dbg
    );

    last_dbg = dbg;
    last_u_cmd = u_cmd;

    // --- (5) Actuator lag ---
    const float u_lagged = actuator.step(u_cmd, dt);

    return Command{u_cmd, u_lagged};
  }
};

} // namespace ab