#include <Arduino.h>
#include <hardware/adc.h>
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <string.h> 

#include "config.h"
#include "flight_core.h"       // The Brain
#include "buffer_manager.h"
#include "sensors.h"
#include "gps.h"
#include "sd_logger.h"
#include "telemetry.h"
#include "state_machine.h"
#include "dma_manager.h"

#define PIN_SERVO 2

// --- Global Instances ---
FlightCore flight;             // Encapsulates Control, Orientation, and Physics
int gps_spinlock_id;
spin_lock_t* gps_spinlock;
RocketState latest_telemetry_packet;

// --- ISR: The 150Hz Heartbeat ---
bool flight_control_isr(struct repeating_timer *t) {
    
    // 1. Hardware: Read Sensors (Body Frame)
    static float ax, ay, az; // m/s^2
    static float gx, gy, gz; // deg/s
    read_imu_calibrated(&ax, &ay, &az, &gx, &gy, &gz);

    static float p_pa = 101325.0f, t_c = 25.0f;
    read_dps368_data(&p_pa, &t_c);
    
    static int16_t mx_raw, my_raw, mz_raw;
    static int mag_counter = 0;
    if (++mag_counter >= 5) { // Read Mag at ~30Hz
        read_mag(&mx_raw, &my_raw, &mz_raw);
        mag_counter = 0;
    }
    float mx = mx_raw * 0.01f;
    float my = my_raw * 0.01f;
    float mz = mz_raw * 0.01f;

    // Read Pitot
    uint16_t adc = dma_adc_get_latest_average();
    // Assuming MPX_OFFSET_V and MPX_SCALE_FACTOR are in config.h
    float pitot_pa = (adc * (3.3f / 4095.0f) - MPX_OFFSET_V) * MPX_SCALE_FACTOR;
    if (pitot_pa < 0) pitot_pa = 0;

    // Hardware: Thread-Safe GPS Snapshot
    GpsState gps_now;
    uint32_t irq = spin_lock_blocking(gps_spinlock);
    memcpy(&gps_now, (void*)&latest_gps, sizeof(GpsState));
    spin_unlock(gps_spinlock, irq);

    // 2. Logic: Update Flight Core
    // This runs the Madgwick Filter, Airspeed Calc, and Control System
    float dt = 1.0f / 150.0f;
    RocketState state = flight.update(dt, ax, ay, az, gx, gy, gz, mx, my, mz, 
                                      p_pa, t_c, pitot_pa, gps_now);

    // 3. System: Log Data
    state_machine_update(&state); // Updates global FSM based on new telemetry
    buffer_push_state(state);

    return true; 
}

void setup() {   
    dma_adc_init(); 
    
    // Init Spinlocks
    gps_spinlock_id = spin_lock_claim_unused(true);
    gps_spinlock = spin_lock_init(gps_spinlock_id);
    
    buffer_manager_init(); // Internally inits buffer spinlock
    
    sensors_init();
    continuity_init();
    gps_init();
    state_machine_init();
    
    // Init The Brain
    flight.begin(PIN_SERVO); // Attaches Servo
    
    Serial.println("Calibrating Ground Pressure...");
    flight.calibrateGroundPressure(50); // Takes ~0.5s

    // Start 150Hz Timer
    static struct repeating_timer timer;
    add_repeating_timer_us(6666, flight_control_isr, NULL, &timer);
}

void loop() {
    gps_ubx_task(); 
    watchdog_update();
}

// --- Core 1: Logging & Telemetry ---
void setup1() {
    delay(2000); 
    sd_logger_init();
    telemetry_init();
}

void loop1() {
    static uint32_t last_telem_ms = 0;
    RocketState queue_item;

    // Empty the buffer to SD Card
    while (buffer_pop_state(queue_item)) { 
        sd_log_data(&queue_item);
        latest_telemetry_packet = queue_item; // Update global for telemetry
    }

    // Telemetry Rate Limiting
    // Low Power (State 0/7): 0.2Hz (5000ms)
    // Flight Mode: 5Hz (200ms) - Adjust as needed
    uint32_t telem_interval = 200;
    bool use_power_save = false;

    if (latest_telemetry_packet.fsm_state == 0 || latest_telemetry_packet.fsm_state == 7) {
        telem_interval = 5000; 
        use_power_save = true;
    }

    if (millis() - last_telem_ms > telem_interval) {
        if (use_power_save) {
            telemetry_set_normal();
            delay(10);             
            telemetry_task(&latest_telemetry_packet);
            delay(100); 
            telemetry_set_sleep();
        } else {
            telemetry_task(&latest_telemetry_packet);
        }
        last_telem_ms = millis();
    }
}