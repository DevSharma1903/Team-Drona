#pragma once
#include "pico/stdlib.h"

#define DEBUG_MODE 

// LSM9DS1 IMU
#define I2C0_SDA_PIN        0
#define I2C0_SCL_PIN        1
#define I2C0_FREQ_HZ        400000
#define LSM9DS1_I2C_ADDR    0x6A

// SPI0: 
#define SPI0_MISO_PIN       24
#define SPI0_MOSI_PIN       27
#define SPI0_SCK_PIN        26
#define SPI0_CS_PIN         25

// SPI1: SD CARD
#define SD_MISO_PIN         32
#define SD_MOSI_PIN         35
#define SD_SCK_PIN          34
#define SD_CS_PIN           33

// UART0: U-BLOX NEO-M9N GPS
#define UART0_TX_PIN        9
#define UART0_RX_PIN        8
#define UART0_BAUD          115200 

// UART1: RYLR993 LORA TELEMETRY
#define UART1_TX_PIN        28
#define UART1_RX_PIN        29
#define UART1_BAUD          9600 

// ADC: MPX Pressure Sensor
#define ADC_DIFFPRESS_PIN   26  // GPIO 26 (ADC0)
#define ADC_CHANNEL_PDUS    0

// PYRO CHANNELS
#define PYRO_DROGUE_PIN     4
#define PYRO_MAIN_PIN       5

// Sensor Calibration Constants
#define MPX_OFFSET_V    0.2f       
#define MPX_SCALE_FACTOR 22222.22f

// Thresholds
#define SENSOR_ISR_FREQ_HZ  150
#define TELEMETRY_RATE_HZ   10

// Buffer Sizes
#define SD_BUFFER_SIZE      128  // Packets

// Physical Constants
#define SEA_LEVEL_PRESSURE  101325.0f
#define MAIN_DEPLOY_ALT_M   457.0f

// Continuity GPIO
#define CONT_DROGUE_ENABLE_PIN 19
#define CONT_MAIN_ENABLE_PIN   20
#define CONT_DROGUE_ADC_PIN_GPIO 40  // ADC1
#define CONT_MAIN_ADC_PIN_GPIO   41  // ADC2

// Continuity ADC Channels
#define ADC_CHAN_DROGUE     0
#define ADC_CHAN_MAIN       1

#define CONT_THRESHOLD      300

// Debug Macros
#ifdef DEBUG_MODE
  #define DEBUG_PRINT(x)       Serial.print(x)
  #define DEBUG_PRINTLN(x)     Serial.println(x)
  #define DEBUG_PRINTF(...)    Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif