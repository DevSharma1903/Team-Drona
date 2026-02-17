#pragma once
#include <cmath>
#include "types.hpp"

namespace ab {

struct ActuatorParams {
  float tau_s;       // time constant
  float min_cmd = 0.0f;
  float max_cmd = 1.0f;
};

struct Actuator1stOrder {
  ActuatorParams p;
  float y = 0.0f;    // current actuator output (0..1)

  explicit Actuator1stOrder(const ActuatorParams& params) : p(params) {}

  float step(float u_cmd, float dt) {
    float u = clampf(u_cmd, p.min_cmd, p.max_cmd);
    if (p.tau_s <= 1e-6f) { y = u; return y; }

    // y_dot = (u - y)/tau
    float dy = (u - y) * (dt / p.tau_s);
    y += dy;
    y = clampf(y, p.min_cmd, p.max_cmd);
    return y;
  }
};

} // namespace ab
