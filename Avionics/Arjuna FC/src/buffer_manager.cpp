#include "flight_data.h"
#include "buffer_manager.h"
#include "hardware/sync.h"

RingBuffer<RocketState, SD_BUFFER_SIZE> sd_buffer;
volatile GpsState latest_gps;
static spin_lock_t* sd_buffer_lock;

void buffer_manager_init() {
    gps_spinlock = spin_lock_init(spin_lock_claim_unused(true));
    sd_buffer_lock = spin_lock_init(spin_lock_claim_unused(true));
    latest_gps.updated = false;
}

void buffer_push_state(const RocketState& state) {
    uint32_t irq_status = spin_lock_blocking(sd_buffer_lock);
    sd_buffer.push(state);
    spin_unlock(sd_buffer_lock, irq_status);
}

bool buffer_pop_state(RocketState& state) {
    uint32_t irq_status = spin_lock_blocking(sd_buffer_lock);
    bool success = sd_buffer.pop(state);
    spin_unlock(sd_buffer_lock, irq_status);
    return success;
}