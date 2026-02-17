#pragma once
#include <stdint.h>
#include "flight_data.h"

bool gps_init();
void gps_ubx_task();
void gps_set_rate(uint8_t hz);
void gps_disable_nmea_output();