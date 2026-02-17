#include "sd_logger.h"
#include "config.h"
#include "buffer_manager.h" 
#include <SPI.h>
#include <SD.h>

static File log_file;
static File event_file;
static bool sd_ok = false;

bool sd_logger_init() {
    SPI1.setRX(SD_MISO_PIN);
    SPI1.setTX(SD_MOSI_PIN);
    SPI1.setSCK(SD_SCK_PIN);
    
    if (!SD.begin(SD_CS_PIN, SPI1)) {
        return false;
    }
    
    char name[15];
    for (int i=0; i<100; i++) {
        sprintf(name, "FLT%02d.BIN", i);
        if (!SD.exists(name)) {
            log_file = SD.open(name, O_WRITE | O_CREAT);
            sprintf(name, "EVT%02d.TXT", i);
            event_file = SD.open(name, O_WRITE | O_CREAT);
            break;
        }
    }
    
    sd_ok = (log_file && event_file);
    return sd_ok;
}

void sd_log_data(RocketState* state) {
    if (sd_ok) {
        log_file.write((uint8_t*)state, sizeof(RocketState));
        
        static int sync = 0;
        if (++sync > 100) { log_file.flush(); sync=0; }
    }
}

void sd_log_event(const char* msg) {
    if (sd_ok) {
        event_file.print(millis());
        event_file.print(": ");
        event_file.println(msg);
        event_file.flush();
    }
}