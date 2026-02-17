#pragma once
#include "flight_data.h"

bool sd_logger_init();
void sd_log_data(RocketState* state);
void sd_log_event(const char* msg);
