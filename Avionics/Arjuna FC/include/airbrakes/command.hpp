#pragma once
#include <cmath>
#include "types.hpp"
#include "model.hpp"
#include "smc.hpp"   // deadband_v, Vec2, smc_reaching_law_u

namespace ab {

// compute_u_ff(pred_no, pred_full, target) with clip [0..1]
static inline float compute_u_ff(float pred_no, float pred_full, float target) {
  const float den = pred_no - pred_full;
  float u_ff = 0.0f;

  if (den > 1e-6f) {
    u_ff = (pred_no - target) / den;
  } else {
    u_ff = 0.0f;
  }

  return clampf(u_ff, 0.0f, 1.0f);
}

// compute_u_trim(...) using st.v instead of v_clone
static inline void compute_u_trim(
    const ModelState& st,
    float Dm_wo,
    float Dm_only,
    float pred_no,
    float target,
    const ModelParams& params,
    const SMCReachingLawParams& smc_params,
    float k_trim,
    float trim_limit,
    float& u_trim_out,
    float& u_smc_out,
    float& s_out
) {
  const float apogee_error = pred_no - target;
  const float v_db = deadband_v(st.v, params.v_eps);

  // x = [apogee_error, v_db]
  const Vec2 x{apogee_error, v_db};

  // fx = [v, Dm_wo]   (v_clone == v)
  const Vec2 fx{st.v, Dm_wo};

  // gx = [0, Dm_only]
  const Vec2 gx{0.0f, Dm_only};

  float s = 0.0f;
  const float u_smc = smc_reaching_law_u(x, fx, gx, smc_params, s);

  // u_trim = clip(k_trim * u_smc, -trim_limit, +trim_limit)
  const float u_trim = clampf(k_trim * u_smc, -trim_limit, +trim_limit);

  u_trim_out = u_trim;
  u_smc_out = u_smc;
  s_out = s;
}

// compose_u_cmd(u_ff, u_trim, u_min, u_max) -> clip(u_ff + u_trim, u_min, u_max)
static inline float compose_u_cmd(float u_ff, float u_trim, float u_min, float u_max) {
  return clampf(u_ff + u_trim, u_min, u_max);
}

// compute_control_command(...) returns u_cmd and fills dbg
static inline float compute_control_command(
    const ModelState& st,
    float pred_no,
    float pred_full,
    float target,
    bool enabled,
    float Dm_wo,
    float Dm_only,
    const ModelParams& params,
    const SMCReachingLawParams& smc_params,
    float k_trim,
    float trim_limit,
    CmdDebug& dbg_out
) {
  CmdDebug dbg;
  dbg.enabled = enabled;
  dbg.err = (pred_no - target);

  if (!enabled) {
    dbg.u_ff = 0.0f;
    dbg.u_smc = 0.0f;
    dbg.u_trim = 0.0f;
    dbg.s = st.s;
    dbg_out = dbg;
    return 0.0f;
  }

  const float u_ff = compute_u_ff(pred_no, pred_full, target);

  float u_trim = 0.0f, u_smc = 0.0f, s = 0.0f;
  compute_u_trim(
      st,
      Dm_wo,
      Dm_only,
      pred_no,
      target,
      params,
      smc_params,
      k_trim,
      trim_limit,
      u_trim,
      u_smc,
      s
  );

  const float u_cmd = compose_u_cmd(u_ff, u_trim, params.u_min, params.u_max);

  dbg.u_ff = u_ff;
  dbg.u_smc = u_smc;
  dbg.u_trim = u_trim;
  dbg.s = s;

  dbg_out = dbg;
  return u_cmd;
}

} // namespace ab
