#pragma once
#include <cmath>
#include <cstdint>
#include "types.hpp"

namespace ab {

// Mirrors airbrakes_smc.models.SMCReachingLawParams for 2-state system.
struct SMCReachingLawParams {
  // Sliding surface vector C = [c0, c1]^T
  float C0 = 0.0f;
  float C1 = 0.0f;

  float eta = 0.0f;
  float alpha = 1.0f;  // should be >= 0 typically
  float K = 0.0f;
  float phi = 0.0f;

  float v_eps = 0.0f;  // deadband threshold for velocity (m/s)
};

// deadband_v(v, v_eps): 0 if |v|<=v_eps else v
static inline float deadband_v(float v, float v_eps) {
  return (std::fabs(v) <= v_eps) ? 0.0f : v;
}

// theta_saturation(s, phi)
static inline float theta_saturation(float s, float phi) {
  if (s > phi)  return  1.0f;
  if (s < -phi) return -1.0f;

  if (phi > 0.0f) return s / phi;

  // phi == 0: sign(s)
  return (s > 0.0f) ? 1.0f : (s < 0.0f ? -1.0f : 0.0f);
}

// reaching_law_h(s, p) = -(eta * |s|^alpha * th) - (K*s)
static inline float reaching_law_h(float s, const SMCReachingLawParams& p) {
  const float th = theta_saturation(s, p.phi);
  const float abs_s = std::fabs(s);

  // Handle abs(s)^alpha robustly
  float pow_term = 0.0f;
  if (abs_s > 0.0f) {
    pow_term = std::pow(abs_s, p.alpha);
  } else {
    // abs_s == 0 => 0^alpha is 0 for alpha>0; treat as 0
    pow_term = 0.0f;
  }

  return -(p.eta * pow_term * th) - (p.K * s);
}

// Minimal 2-state vectors for embedded determinism
struct Vec2 {
  float a0 = 0.0f;
  float a1 = 0.0f;
};

// Compute dot(C, x)
static inline float dotC(const SMCReachingLawParams& p, const Vec2& v) {
  return p.C0 * v.a0 + p.C1 * v.a1;
}

/*
  smc_reaching_law_u(x, fx, gx, p):

  u = (C^T g(x))^-1 ( -C^T f(x) + h(s) ), s = C^T x

  Inputs:
    x  = [h_error, v_error] or whatever your controller state uses (2x1)
    fx = f(x) as 2x1
    gx = g(x) as 2x1   (NOTE: in your Python Ct_g is scalar, so gx is 2x1)
*/
static inline float smc_reaching_law_u(
    const Vec2& x_in,
    const Vec2& fx_in,
    const Vec2& gx_in,
    const SMCReachingLawParams& p,
    float& s_out
) {
  // Apply deadband to the velocity component of x and also to fx/gx velocity term
  // (This mirrors your pipeline behavior if you deadband v before forming x/fx/gx)
  Vec2 x = x_in;
  Vec2 fx = fx_in;
  Vec2 gx = gx_in;

  x.a1  = deadband_v(x.a1,  p.v_eps);
  fx.a1 = deadband_v(fx.a1, p.v_eps);
  gx.a1 = deadband_v(gx.a1, p.v_eps);

  const float s = dotC(p, x);
  const float Ct_f = dotC(p, fx);
  const float Ct_g = dotC(p, gx);

  s_out = s;

  if (std::fabs(Ct_g) < 1e-12f) {
    return 0.0f;
  }

  const float u = (-Ct_f + reaching_law_h(s, p)) / Ct_g;
  return u;
}

} // namespace ab
