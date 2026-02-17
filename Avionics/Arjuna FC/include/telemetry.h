#pragma once
#include <stdint.h>
#include "flight_data.h"

bool telemetry_init();

void telemetry_task(RocketState* state);

void telemetry_set_sleep();
void telemetry_set_normal();
