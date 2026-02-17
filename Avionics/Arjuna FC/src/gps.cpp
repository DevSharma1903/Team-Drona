#include "gps.h"
#include "config.h"
#include "buffer_manager.h"
#include <Arduino.h>

#define GPSSerial Serial1

const uint8_t UBX_HEADER[] = {0xB5, 0x62};
const uint8_t CLASS_NAV = 0x01;
const uint8_t ID_PVT    = 0x07;

enum UbxState { WAIT_SYNC1, WAIT_SYNC2, WAIT_CLASS, WAIT_ID, WAIT_LEN1, WAIT_LEN2, WAIT_PAYLOAD, WAIT_CKA, WAIT_CKB };
static UbxState state = WAIT_SYNC1;
static uint8_t msg_class, msg_id;
static uint16_t msg_len;
static uint16_t payload_idx;
static uint8_t ck_a, ck_b, rx_ck_a;
static uint8_t payload[100]; 

uint32_t extract_u32(uint8_t* buf, uint8_t offset) {
    uint32_t res; memcpy(&res, &buf[offset], 4); return res;
}
int32_t extract_i32(uint8_t* buf, uint8_t offset) {
    int32_t res; memcpy(&res, &buf[offset], 4); return res;
}

void send_ubx(uint8_t* cmd, uint8_t len) {
    for(int i=0; i<len; i++) GPSSerial.write(cmd[i]);
    GPSSerial.flush();
}

void gps_disable_nmea_output() {
    
    uint8_t packet[] = {
        0xB5, 0x62,             
        0x06, 0x8A,            
        0x0E, 0x00,             
        0x00,                   
        0x01,                   
        0x00, 0x00,           
        0x01, 0x00, 0x74, 0x10, 
        0x01,                   
        0x02, 0x00, 0x74, 0x10, 
        0x00,                   
        0x8D, 0x5D 
    };
    
    send_ubx(packet, sizeof(packet));
    DEBUG_PRINTLN("[GPS] NMEA Output Disabled (UBX Only)");
}

bool gps_init() {
    DEBUG_PRINT("[GPS] Init UBX @ 115200... ");
    GPSSerial.setTX(UART0_TX_PIN);
    GPSSerial.setRX(UART0_RX_PIN);
    GPSSerial.setFIFOSize(256);
    GPSSerial.begin(UART0_BAUD);
    delay(100);
    gps_disable_nmea_output();
    DEBUG_PRINTLN("OK");
    return true;
}

void gps_set_low_power() {
    uint8_t packet[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 
        0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 
        0x01, 0x39
    };
    send_ubx(packet, sizeof(packet));
    DEBUG_PRINTLN("[GPS] Set to 1Hz");
}

void gps_set_flight_mode() {
    uint8_t packet_20hz[] = {
        0xB5, 0x62, 
        0x06, 0x08, 
        0x06, 0x00, 
        0x32, 0x00,
        0x01, 0x00, 
        0x01, 0x00, 
        0x48, 0xE6 
    };
    send_ubx(packet_20hz, sizeof(packet_20hz));
    DEBUG_PRINTLN("[GPS] Set to 20Hz");
}

void gps_ubx_task() {
    while (GPSSerial.available()) {
        uint8_t c = GPSSerial.read();
        
        switch (state) {
            case WAIT_SYNC1: if(c==0xB5) state=WAIT_SYNC2; break;
            case WAIT_SYNC2: if(c==0x62) {state=WAIT_CLASS; ck_a=0; ck_b=0;} else state=WAIT_SYNC1; break;
            case WAIT_CLASS: msg_class=c; ck_a+=c; ck_b+=ck_a; state=WAIT_ID; break;
            case WAIT_ID:    msg_id=c;    ck_a+=c; ck_b+=ck_a; state=WAIT_LEN1; break;
            case WAIT_LEN1:  msg_len=c;   ck_a+=c; ck_b+=ck_a; state=WAIT_LEN2; break;
            case WAIT_LEN2:  msg_len|=(c<<8); ck_a+=c; ck_b+=ck_a; payload_idx=0; state=WAIT_PAYLOAD; break;
            case WAIT_PAYLOAD:
                if(payload_idx < sizeof(payload)) payload[payload_idx] = c;
                payload_idx++;
                ck_a+=c; ck_b+=ck_a;
                if(payload_idx == msg_len) state=WAIT_CKA;
                break;
            case WAIT_CKA: rx_ck_a=c; state=WAIT_CKB; break;
            case WAIT_CKB: 
                if(ck_a==rx_ck_a && ck_b==c) {
                    if(msg_class==CLASS_NAV && msg_id==ID_PVT) {
                        GpsState new_fix;
                        new_fix.iTOW     = extract_u32(payload, 0);
                        new_fix.fix_type = payload[20];
                        new_fix.num_sats = payload[23];
                        new_fix.lon      = extract_i32(payload, 24);
                        new_fix.lat      = extract_i32(payload, 28);
                        new_fix.alt_msl  = extract_i32(payload, 36);
                        new_fix.updated  = true;

                        uint32_t irq = spin_lock_blocking(gps_spinlock);
                        memcpy((void*)&latest_gps, &new_fix, sizeof(GpsState));
                        spin_unlock(gps_spinlock, irq);
                    }
                }
                state=WAIT_SYNC1; 
                break;
        }
    }
}