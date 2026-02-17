#include "telemetry.h"
#include "config.h"
#include <Arduino.h>

#define LoRaSerial Serial2

bool telemetry_init() {
    LoRaSerial.setTX(UART1_TX_PIN);
    LoRaSerial.setRX(UART1_RX_PIN);
    LoRaSerial.begin(UART1_BAUD); 
    
    delay(1000);
    
    LoRaSerial.println("AT+OPMODE=1");
    delay(500);
    
    LoRaSerial.println("ATZ");
    delay(2000);
    
    LoRaSerial.println("AT+CRFOP=22");
    delay(100);

    LoRaSerial.println("AT+ADDRESS=1");
    delay(100);
    
    LoRaSerial.println("AT+PARAMETER=7,7,1,12"); 
    delay(100);
    
    DEBUG_PRINTLN("[TELEM] RYLR993 Init OK (Mode 1)");
    
    return true; 
}

void telemetry_set_sleep() {
    LoRaSerial.println("AT+MODE=1"); 
}

void telemetry_set_normal() {
    LoRaSerial.println("AT+MODE=0"); 
}

void telemetry_task(RocketState* state) {
    if (state == nullptr) return;

    char buffer[200]; 
    
    int len = sizeof(RocketState) * 2;
    int offset = sprintf(buffer, "AT+SEND=0,%d,", len);
    
    uint8_t* raw_bytes = (uint8_t*)state;
    
    for (size_t i = 0; i < sizeof(RocketState); i++) {
        sprintf(&buffer[offset + (i * 2)], "%02X", raw_bytes[i]);
    }
    
    LoRaSerial.println(buffer);
}