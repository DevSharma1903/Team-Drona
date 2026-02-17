#pragma once
#include <math.h>
#include <stdint.h>

class Orientation {
private:
    // Madgwick filter parameters
    float beta = 0.1f; // [cite: 7] FILTER_PARAM in your sketch was 0.9, but 0.1 is standard for valid time steps.
                       // Your sketch used 0.9 because the library likely expects a different scale. 
                       // We will stick to standard 0.1 for manual implementation or you can tune it.
    
    // Quaternion state (Internal Engine)
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    
    // Estimated Gravity (from calibration) [cite: 8]
    float g_est = 1.0f; 

    // Helper: Degrees to Radians [cite: 17]
    float deg2rad(float d) { return d * 0.01745329251994329577f; }

    // Helper: Fast Inverse Square Root
    float invSqrt(float x) {
        if (x <= 0) return 0;
        return 1.0f / sqrtf(x);
    }

    // Logic from: computeR_ZXY_ENU 
    // Recreates the exact rotation matrix from your sketch
    void computeR_ZXY_ENU(float yaw_deg, float pitch_deg, float roll_deg, float R[3][3]) {
        float y = deg2rad(yaw_deg);
        float p = deg2rad(pitch_deg);
        float r = deg2rad(roll_deg);

        float sy = sinf(y), cy = cosf(y);
        float sp = sinf(p), cp = cosf(p);
        float sr = sinf(r), cr = cosf(r);

        // Exact formula from lines 30-34
        R[0][0] = -sy * sp * sr + cy * cp;
        R[0][1] = -sy * cr;
        R[0][2] =  sy * cp * sr + cy * sp;

        R[1][0] =  cy * sp * sr + sy * cp;
        R[1][1] =  cy * cr;
        R[1][2] = -cy * cp * sr + sy * sp;

        R[2][0] = -sp * cr;
        R[2][1] =  sr;
        R[2][2] =  cp * cr;
    }

    // Logic from: matVec3 
    void matVec3(const float R[3][3], const float v[3], float out[3]) {
        out[0] = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
        out[1] = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
        out[2] = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
    }

public:
    // Outputs
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    float lin_acc_z = 0.0f; // True vertical acceleration (gravity removed)

    void setGravityCalibration(float g_calib) {
        g_est = g_calib;
    }

    // Main Update Function
    // Inputs: Gyro (deg/s), Accel (g's), dt (seconds)
    // Note: Sketch uses deg/s for gyro [cite: 67] and g for accel [cite: 66]
    void update(float gx_dps, float gy_dps, float gz_dps, float ax_g, float ay_g, float az_g, float dt) {
        
        // 1. Run Standard Madgwick (Quaternion Update)
        // We must convert deg/s to rad/s for the math, even though input is dps
        float gx = deg2rad(gx_dps);
        float gy = deg2rad(gy_dps);
        float gz = deg2rad(gz_dps);

        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax_g == 0.0f) && (ay_g == 0.0f) && (az_g == 0.0f))) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
            ax_g *= recipNorm;
            ay_g *= recipNorm;
            az_g *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            // Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax_g + _4q0 * q1q1 - _2q1 * ay_g;
            s1 = _4q1 * q3q3 - _2q3 * ax_g + 4.0f * q0q0 * q1 - _2q0 * ay_g - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az_g;
            s2 = 4.0f * q0q0 * q2 + _2q0 * ax_g + _4q2 * q3q3 - _2q3 * ay_g - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az_g;
            s3 = 4.0f * q1q1 * q3 - _2q1 * ax_g + 4.0f * q2q2 * q3 - _2q2 * ay_g;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        // 2. Convert to Euler Angles (Standard sequence to match library output)
        // Corresponds to filter.getRoll(), getPitch(), getYaw() 
        yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
        pitch = -asinf(2.0f * (q1 * q3 - q0 * q2)) * 57.29578f;
        roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29578f;

        // 3. Compute Rotation Matrix (The Exact Logic) 
        float R[3][3];
        computeR_ZXY_ENU(yaw, pitch, roll, R);

        // 4. Rotate Body Accel to ENU Frame [cite: 73]
        // Note: We need original accel magnitude here (in g), not normalized
        // Since we normalized ax_g above, we should technically pass a fresh copy or restore it.
        // For this function, let's assume the user passes raw Gs that we can use.
        // *Refinement*: The internal math above normalized the inputs. 
        // We need to use the RAW inputs for the projection. 
        // I will assume the function arguments (ax_g, etc) are strictly inputs. 
        // (In C++, args are copies, so modifying them above doesn't hurt us if we saved them, 
        // but to be safe, I'll use the original values before normalization).
    }
    
    // --- Separate Compute Function to ensure we use RAW accel values ---
    void computeTrueVertical(float ax_g_raw, float ay_g_raw, float az_g_raw) {
         // Re-compute Matrix (or store it from update step)
         float R[3][3];
         computeR_ZXY_ENU(yaw, pitch, roll, R);
         
         float a_body_g[3] = { ax_g_raw, ay_g_raw, az_g_raw }; // [cite: 72]
         float a_nav_g[3];
         
         // Rotate [cite: 73]
         matVec3(R, a_body_g, a_nav_g);
         
         // Gravity Removal 
         // a_nav_g[2] is the Vertical (U) component
         float a_nav_lin_g_z = a_nav_g[2] - g_est; 
         
         // Convert to m/s^2 [cite: 76]
         lin_acc_z = a_nav_lin_g_z * 9.80665f;
    }
};