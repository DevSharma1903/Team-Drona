#pragma once
#include <cstddef>
#include <cmath>

// ============================================================
// EKF for rocket vertical channel (h, v, b)
// Header-only, no STL containers required.
// ============================================================

namespace rocket {

struct EKF3 {
  // -----------------------------
  // Public types
  // -----------------------------
  struct Vec3 {
    float x0, x1, x2; // [h, v, b]
  };

  struct Mat3 {
    float a00, a01, a02;
    float a10, a11, a12;
    float a20, a21, a22;
  };

  struct Mat1 {
    float s00; // scalar 1x1
  };

  // -----------------------------
  // Parameters/constants
  // -----------------------------
  float g = 9.80665f;
  float pi = 3.14f;

  // Geometry
  float diameter_m = 0.126f;      // meters
  float area_m2    = 0.0f;        // computed in init()

  // Masses
  float m_boost_kg = 44.47f;
  float m_coast_kg = 33.64f;

  // Covariance base factor (from your data)
  float cov = 114.419f;

  // Measurement noise (R)
  float R = 100.0f; // altitude measurement variance

  // Bias process noise (third state)
  float q_bias = 0.001f;

  // Q scaling (like your cov*5 and cov*3)
  float launch_scale = 5.0f;
  float coast_scale  = 3.0f;

  // -----------------------------
  // Mach->Cd lookup table pointers
  // (Provide these from your code; stored as pointers only.)
  // -----------------------------
  const float* mach_LUT = nullptr;
  const float* cd_LUT   = nullptr;
  std::size_t  lut_len  = 0;

  // -----------------------------
  // EKF state
  // -----------------------------
  Vec3 X {0.0f, 0.0f, 0.0f};  // [h, v, b]
  Mat3 P {
    100.0f, 0.0f,   0.0f,
    0.0f,   100.0f, 0.0f,
    0.0f,   0.0f,   1.0f
  };

  // Current Q (3x3)
  Mat3 Q {0};

  // -----------------------------
  // Init / configuration
  // -----------------------------
  void init() {
    // Area = pi * (d/2)^2
    const float r = 0.5f * diameter_m;
    area_m2 = pi * r * r;

    // Default Q at dt=1 will be replaced per-step using actual dt
    Q = makeQ(/*dt=*/0.027f, /*cov_scale=*/1.0f);
  }

  void setLUT(const float* mach, const float* cd, std::size_t n) {
    mach_LUT = mach;
    cd_LUT   = cd;
    lut_len  = n;
  }

  void setInitialState(float h0, float v0, float b0) {
    X = {h0, v0, b0};
  }

  void setInitialCovariance(float phh, float pvv, float pbb) {
    P = {
      phh, 0.0f, 0.0f,
      0.0f, pvv, 0.0f,
      0.0f, 0.0f, pbb
    };
  }

  // -----------------------------
  // Main step
  // Inputs:
  //  state_flag: 1=boost, 2=coast, else default
  //  altitude_m: Altitude_P
  //  accY_mps2:  AccY (sensor vertical accel)
  //  dt_s:       time step
  // Returns: estimated velocity (m/s)
  // -----------------------------
  float step(int state_flag, float altitude_m, float accY_mps2, float dt_s, float t_boost) {
    if (dt_s <= 0.0f) {
      update(altitude_m);
      return X.x1;
    }

    const float sound = 340.3f - 0.003f * altitude_m;
    const float sound_safe = (sound > 1.0f) ? sound : 1.0f;
    const float rho = densityISA(altitude_m);
    const float u = (accY_mps2 - g);
    const float mach = X.x1 / sound_safe;
    const float Cd = lookupCd(mach);

    // 2. Determine base mass and Q scale
    float m_base = (state_flag == 1) ? get_dynamic_mass(t_boost) : m_coast_kg;
    float cov_scale = (state_flag == 1) ? launch_scale : (state_flag == 2 ? coast_scale : 1.0f);

    // 3. Dynamic ballistic coefficient calculation
    const float k = (rho * Cd * area_m2) / (2.0f * m_base);

    Q = makeQ(dt_s, cov_scale);

    // 4. Pass t_boost to predict
    predict(u, k, dt_s, t_boost); 
    update(altitude_m);

    return X.x1;
  }

private:
  // -----------------------------
  // Helper: ISA density (your exact formula)
  // -----------------------------
  float densityISA(float h_m) const {
    // Clamp factor base to avoid pow of negative
    const float base = 1.0f - (0.0065f * h_m / 288.15f);
    const float base_safe = (base > 1e-6f) ? base : 1e-6f;
    const float expn = (g / (287.058f * 0.0065f)) - 1.0f;
    return 1.225f * std::pow(base_safe, expn);
  }

  // -----------------------------
  // Helper: linear interpolation (np.interp equivalent)
  // If LUT not set, returns a safe fallback Cd.
  // -----------------------------
  float lookupCd(float mach) const {
    if (!mach_LUT || !cd_LUT || lut_len < 2) {
      return 0.5f; // fallback if no LUT
    }

    // If mach outside range, clamp to edges
    if (mach <= mach_LUT[0]) return cd_LUT[0];
    if (mach >= mach_LUT[lut_len - 1]) return cd_LUT[lut_len - 1];

    // Find interval (linear search; for small LUTs this is fine)
    // If LUT is large and FC is tight, switch to binary search.
    std::size_t i = 0;
    while (i + 1 < lut_len && mach_LUT[i + 1] < mach) {
      ++i;
    }

    const float x0 = mach_LUT[i];
    const float x1 = mach_LUT[i + 1];
    const float y0 = cd_LUT[i];
    const float y1 = cd_LUT[i + 1];

    const float denom = (x1 - x0);
    if (std::fabs(denom) < 1e-9f) return y0;

    const float t = (mach - x0) / denom;
    return y0 + t * (y1 - y0);
  }

  // -----------------------------
  // Build Q like:
  // [[0.25*dt^4*cov, 0.5*dt^3*cov, 0],
  //  [0.5*dt^3*cov,  dt^2*cov,     0],
  //  [0,             0,            0.001]]
  // with cov scaled by cov_scale.
  // -----------------------------
  Mat3 makeQ(float dt, float cov_scale) const {
    const float c = cov * cov_scale;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    Mat3 q;
    q.a00 = 0.25f * dt4 * c;  q.a01 = 0.5f * dt3 * c;   q.a02 = 0.0f;
    q.a10 = 0.5f * dt3 * c;   q.a11 = dt2 * c;          q.a12 = 0.0f;
    q.a20 = 0.0f;             q.a21 = 0.0f;             q.a22 = q_bias;
    return q;
  }

  // -----------------------------
  // Dynamics f(X)
  // -----------------------------
  Vec3 fstate(const Vec3& x, float u, float k, float dt) const {
    const float h = x.x0;
    const float v = x.x1;
    const float b = x.x2;

    const float h_new = h + v * dt;
    const float v_new = v + (u - b - k * std::fabs(v) * v) * dt;
    const float b_new = b;

    return {h_new, v_new, b_new};
  }

  // -----------------------------
  // Jacobian F = df/dX
  // Your python:
  // [[1, dt, 0],
  //  [0, 1-2*k*abs(v)*dt, -dt],
  //  [0, 0, 1]]
  // -----------------------------
  Mat3 F_jac(const Vec3& x, float k, float dt) const {
    const float v = x.x1;
    Mat3 F;
    F.a00 = 1.0f;  F.a01 = dt;                               F.a02 = 0.0f;
    F.a10 = 0.0f;  F.a11 = 1.0f - 2.0f * k * std::fabs(v) * dt; F.a12 = -dt;
    F.a20 = 0.0f;  F.a21 = 0.0f;                              F.a22 = 1.0f;
    return F;
  }

  // -----------------------------
  // Predict step:
  // X = f(X)
  // P = F P F^T + Q
  // -----------------------------
  // Function logic is correct
  inline float get_dynamic_mass(float t_boost) {
    if (t_boost <= 0.0f) return m_boost_kg;
    if (t_boost >= 4.0f) return m_coast_kg;

    float m_dynamic = (0.0196336f * t_boost * t_boost) + (-2.85696f * t_boost) + 44.6794f;
    return (m_dynamic < m_coast_kg) ? m_coast_kg : m_dynamic;
  }

  void predict(float u, float k, float dt, float t_boost) {
    // Note: 'k' already contains the dynamic mass effect from the step() calculation
    X = fstate(X, u, k, dt);

    const Mat3 F = F_jac(X, k, dt);
    const Mat3 FP  = mul(F, P);
    const Mat3 FPFt = mul(FP, transpose(F));
    P = add(FPFt, Q);
  }

  // -----------------------------
  // Measurement model:
  // z = h
  // H = [1, 0, 0]
  // Update:
  // S = H P H^T + R = P00 + R
  // y = z - h
  // K = P H^T S^-1 = [P00, P10, P20]^T / S
  // X = X + K*y
  // P = (I - K H) P
  // -----------------------------
  void update(float z_altitude) {
    // Innovation
    const float y = z_altitude - X.x0;

    // S = P00 + R
    const float S = P.a00 + R;
    const float invS = (std::fabs(S) > 1e-9f) ? (1.0f / S) : 0.0f;

    // K = [P00, P10, P20]^T / S
    const float K0 = P.a00 * invS;
    const float K1 = P.a10 * invS;
    const float K2 = P.a20 * invS;

    // State update
    X.x0 += K0 * y;
    X.x1 += K1 * y;
    X.x2 += K2 * y;

    // P update: P = (I - K H) P
    // With H = [1,0,0], (I - K H) =
    // [[1-K0, 0, 0],
    //  [-K1,  1, 0],
    //  [-K2,  0, 1]]
    // Multiply: P_new = A * P_old
    const Mat3 P_old = P;

    Mat3 A;
    A.a00 = 1.0f - K0; A.a01 = 0.0f; A.a02 = 0.0f;
    A.a10 = -K1;       A.a11 = 1.0f; A.a12 = 0.0f;
    A.a20 = -K2;       A.a21 = 0.0f; A.a22 = 1.0f;

    P = mul(A, P_old);
  }

  // -----------------------------
  // Tiny 3x3 matrix ops
  // -----------------------------
  static Mat3 add(const Mat3& A, const Mat3& B) {
    return {
      A.a00 + B.a00, A.a01 + B.a01, A.a02 + B.a02,
      A.a10 + B.a10, A.a11 + B.a11, A.a12 + B.a12,
      A.a20 + B.a20, A.a21 + B.a21, A.a22 + B.a22
    };
  }

  static Mat3 transpose(const Mat3& A) {
    return {
      A.a00, A.a10, A.a20,
      A.a01, A.a11, A.a21,
      A.a02, A.a12, A.a22
    };
  }

  static Mat3 mul(const Mat3& A, const Mat3& B) {
    Mat3 C;
    C.a00 = A.a00*B.a00 + A.a01*B.a10 + A.a02*B.a20;
    C.a01 = A.a00*B.a01 + A.a01*B.a11 + A.a02*B.a21;
    C.a02 = A.a00*B.a02 + A.a01*B.a12 + A.a02*B.a22;

    C.a10 = A.a10*B.a00 + A.a11*B.a10 + A.a12*B.a20;
    C.a11 = A.a10*B.a01 + A.a11*B.a11 + A.a12*B.a21;
    C.a12 = A.a10*B.a02 + A.a11*B.a12 + A.a12*B.a22;

    C.a20 = A.a20*B.a00 + A.a21*B.a10 + A.a22*B.a20;
    C.a21 = A.a20*B.a01 + A.a21*B.a11 + A.a22*B.a21;
    C.a22 = A.a20*B.a02 + A.a21*B.a12 + A.a22*B.a22;
    return C;
  }
};

} // namespace rocket
