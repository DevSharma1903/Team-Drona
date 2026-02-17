#pragma once
#include <cstdint>
#include <cmath>

namespace ab {

struct State {
  float h_m;     // altitude (m)
  float v_mps;   // vertical velocity (m/s) (up positive)
};

struct EstimatorInput {
  // Provide whatever your estimator gives you each tick:
  // (h, v) is enough for this controller stack.
  float h_m;
  float v_mps;
  float rho_kgm3;

  // Optional (if you have it):
  float a_mps2;     // vertical accel (m/s^2)
  float mach;       // if you already compute it
};

struct Command {
  float u;          // controller output (generic control signal)
  float brake;      // actuator command [0..1] (0=retracted, 1=full)
};

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi ? hi : x);
}

static inline float signf(float x) {
  return (x > 0.0f) - (x < 0.0f);
}

} // namespace ab
