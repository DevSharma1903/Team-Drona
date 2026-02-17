#pragma once
#include <Arduino.h>
#include <hardware/sync.h>
#include "flight_data.h"
#include "config.h"

template <typename T, uint16_t Size>
class RingBuffer {
private:
    T buffer[Size];
    volatile uint16_t head;
    volatile uint16_t tail; 

public:
    void init() {
        head = 0;
        tail = 0;
    }

    bool push(const T& item) {
        uint16_t next = (head + 1) % Size;
        if (next == tail) return false;
        
        buffer[head] = item;
        __dmb();
        head = next;
        return true;
    }

    bool pop(T& item) {
        if (head == tail) return false; 
        
        item = buffer[tail];
        __dmb();
        tail = (tail + 1) % Size;
        return true;
    }

    bool isEmpty() const {
        return head == tail;
    }
    
    bool peekLast(T& item) const {
        if (head == tail) return false;
        uint16_t idx = (head == 0) ? Size - 1 : head - 1;
        item = buffer[idx];
        return true;
    }
};

extern RingBuffer<RocketState, SD_BUFFER_SIZE> sd_buffer;
extern volatile GpsState latest_gps;
extern spin_lock_t* gps_spinlock;

void buffer_manager_init();
void buffer_push_state(const RocketState& state);
bool buffer_pop_state(RocketState& state);