#pragma once
#include <cstdint>

namespace ab {

struct ModelState {
  float h = 0.0f;   // altitude (m) (if you store it)
  float v = 0.0f;   // vertical velocity (m/s)
  float s = 0.0f;   // last sliding surface value (optional, for debug continuity)
};

struct ModelParams {
  float v_eps = 0.0f;  // deadband threshold for velocity
  float u_min = 0.0f;  // min command
  float u_max = 1.0f;  // max command
};

// Mirrors your CmdDebug fields used in compute_control_command
struct CmdDebug {
  bool enabled = false;
  float err = 0.0f;     // pred_no - target

  float u_ff = 0.0f;
  float u_smc = 0.0f;
  float u_trim = 0.0f;
  float s = 0.0f;
};

} // namespace ab
