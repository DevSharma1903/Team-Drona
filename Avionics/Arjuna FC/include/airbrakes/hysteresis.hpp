#pragma once
#include "types.hpp"

namespace ab {

struct Hysteresis {
  float on_threshold_m = 15.0f;
  float off_threshold_m = 5.0f;
  bool enabled = false;

  bool update(float error_m) {
    // If thresholds are misconfigured, degrade gracefully
    const float on_th  = on_threshold_m;
    const float off_th = (off_threshold_m <= on_threshold_m) ? off_threshold_m : on_threshold_m;

    if (!enabled) {
      if (error_m > on_th) enabled = true;
    } else {
      if (error_m < off_th) enabled = false;
    }
    return enabled;
  }
};

} // namespace ab
