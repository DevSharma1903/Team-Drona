#pragma once
#include <cmath>

namespace ab {

struct Atmosphere {
  // Simple troposphere (up to ~11km). Good for a 9-10km rocket.
  static void isa(float h_m, float& rho, float& a_sound) {
    // ISA constants
    constexpr float T0 = 288.15f;      // K
    constexpr float P0 = 101325.0f;    // Pa
    constexpr float g0 = 9.80665f;     // m/s^2
    constexpr float L  = 0.0065f;      // K/m
    constexpr float R  = 287.058f;     // J/(kg*K)
    constexpr float gamma = 1.4f;

    float h = (h_m < 0.0f) ? 0.0f : h_m;
    if (h > 11000.0f) h = 11000.0f;

    float T = T0 - L * h;
    float P = P0 * std::pow(T / T0, g0 / (R * L));
    rho = P / (R * T);
    a_sound = std::sqrt(gamma * R * T);
  }
};

} // namespace ab
