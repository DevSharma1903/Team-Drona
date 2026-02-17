#pragma once
#include <stdint.h>
#include <Arduino.h>

bool sensors_init();

void read_imu_fast(int16_t* ax, int16_t* ay, int16_t* az, 
                   int16_t* gx, int16_t* gy, int16_t* gz);

void read_imu_calibrated(float* ax, float* ay, float* az, 
                         float* gx, float* gy, float* gz);

void read_mag(int16_t* mx, int16_t* my, int16_t* mz);

void write_i2c_reg(uint8_t addr, uint8_t reg, uint8_t val) ;
void sensors_set_low_power();
void sensors_set_flight_mode();
void read_dps368_data(float* pressure_pa, float* temp_celsius);

void continuity_init();
void read_continuity(uint8_t* drogue_status, uint8_t* main_status);
