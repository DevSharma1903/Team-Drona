#include "flight_core.h"
#include "state_machine.h"
#include "sensors.h" 
#include "airbrakes/atmosphere.hpp" //

void FlightCore::begin(int servo_pin) {
    sys.begin();
    airbrakeServo.attach(servo_pin);
    airbrakeServo.write(0); 
}

void FlightCore::calibrateGroundPressure(int samples) {
    float p_sum = 0;
    float t_sum = 0;
    
    for(int i=0; i<samples; i++) {
        float p, t;
        read_dps368_data(&p, &t); 
        p_sum += p;
        t_sum += t;
        delay(10);
    }
    
    if (samples > 0) {
        ground_pressure_pa = p_sum / samples;
        ground_temp_k = Physics::celsius_to_kelvin(t_sum / samples);
    }
}

RocketState FlightCore::update(float dt, 
                               float ax, float ay, float az, 
                               float gx, float gy, float gz,
                               float mx, float my, float mz,
                               float pressure_pa, float temp_c,
                               float pitot_pa,
                               const GpsState& gps_now) 
{
    // 1. Math: Orientation
    float ax_g = Physics::ms2_to_g(ax);
    float ay_g = Physics::ms2_to_g(ay);
    float az_g = Physics::ms2_to_g(az);

    attitude.update(gx, gy, gz, ax_g, ay_g, az_g, dt);
    attitude.computeTrueVertical(ax_g, ay_g, az_g);
    float true_acc_z = attitude.lin_acc_z; 

    // 2. Math: Altitude
    float current_alt_m = Physics::altitude_from_pressure(pressure_pa, ground_pressure_pa, ground_temp_k);
    
    // 3. Control System Update
    // We update this EARLY to get the latest Velocity estimate for Mach calculations
    int fsm_state = get_current_state();
    sys.update(true_acc_z, current_alt_m, dt, fsm_state);

    // 4. Physics: Atmosphere & Mach
    // Use the single source of truth for Atmosphere
    float rho = 0.0f;
    float a_sound = 0.0f;
    ab::Atmosphere::isa(sys.estimated_h, rho, a_sound);

    // Calculate Mach using EKF Velocity (Matches Python Logic)
    mach_number = Physics::calculate_mach_from_velocity(sys.estimated_v, a_sound);
    
    // Calculate TAS
    airspeed_tas = Physics::calculate_tas(mach_number, a_sound);

    static uint32_t last_cont_check = 0;
    static uint8_t cont_drogue = 0;
    static uint8_t cont_main = 0;

    if (millis() - last_cont_check > 1000) {
        read_continuity(&cont_drogue, &cont_main);
        last_cont_check = millis();
    }

    // 5. Actuation
    int servo_angle = (int)(sys.servo_cmd * 90.0f);
    airbrakeServo.write(servo_angle);

    // 6. Packet Population
    RocketState state;
    state.timestamp_ms = millis();
    state.fsm_state    = (uint8_t)fsm_state;

    // Filtered Control Data
    state.altitude           = sys.estimated_h;      
    state.velocity           = sys.estimated_v;      
    state.acc_z_earth_frame  = true_acc_z;
    state.airbrake_extension = sys.servo_cmd;        

    // Physics Data
    state.mach_number  = mach_number;
    state.airspeed_tas = airspeed_tas;

    // Raw Sensors
    state.ax_body = ax; state.ay_body = ay; state.az_body = az;
    state.gx_deg = gx;  state.gy_deg = gy;  state.gz_deg = gz;
    state.mx_ut = mx;   state.my_ut = my;   state.mz_ut = mz;
    
    state.pressure_pa      = pressure_pa;
    state.temp_c           = temp_c;
    state.diff_pressure_pa = pitot_pa;

    state.drogue_continuity = cont_drogue;
    state.main_continuity   = cont_main;

    state.gps_lat  = gps_now.lat;
    state.gps_lon  = gps_now.lon;
    state.gps_alt  = (float)gps_now.alt_msl / 1000.0f; 
    state.gps_iTOW = gps_now.iTOW;
    state.gps_sats = gps_now.num_sats;
    state.gps_fix  = gps_now.fix_type;

    state.tilt_angle_deg = attitude.pitch; 

    return state;
}