#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_DPS310.h>
#include <hardware/adc.h>

static Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
static Adafruit_DPS310 dps;

static bool lsm_ready = false;
static bool dps_ready = false;

static float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

#define LSM9DS1_XG_MEMS_ADDRESS     0x6A
#define LSM9DS1_REGISTER_CTRL_REG1_G 0x10 
#define LSM9DS1_REGISTER_CTRL_REG6_XL 0x20 
#define LSM9DS1_MAG_ADDRESS         0x1C
#define LSM9DS1_REGISTER_CTRL_REG3_M 0x22

#define ACCEL_SCALE (0.732f * 9.80665f / 1000.0f)
#define GYRO_SCALE (0.070f)

bool sensors_init() {
    bool success = true;
    DEBUG_PRINTLN("[SENSORS] Initializing...");

    Wire.setSDA(I2C0_SDA_PIN);
    Wire.setSCL(I2C0_SCL_PIN);
    Wire.begin();
    Wire.setClock(I2C0_FREQ_HZ); 

    if (lsm.begin()) {
        lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_16G);
        lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_2000DPS);
        lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_16GAUSS);
        lsm_ready = true;
        DEBUG_PRINTLN("  - IMU: OK");
    } else {
        DEBUG_PRINTLN("  - IMU: FAILED");
        success = false;
    }

    SPI.setRX(SPI0_MISO_PIN);
    SPI.setTX(SPI0_MOSI_PIN);
    SPI.setSCK(SPI0_SCK_PIN);
    SPI.begin();

    if (dps.begin_SPI(SPI0_CS_PIN, &SPI)) {
        dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
        dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
        dps_ready = true;
        DEBUG_PRINTLN("  - BARO: OK");
    } else {
        DEBUG_PRINTLN("  - BARO: FAILED");
        success = false;
    }

    if (lsm_ready) {
        DEBUG_PRINT("  - Calibrating Gyro... ");
        float gx_sum = 0, gy_sum = 0, gz_sum = 0;
        for (int i = 0; i < 50; i++) {
            lsm.readGyro();
            gx_sum += lsm.gyroData.x;
            gy_sum += lsm.gyroData.y;
            gz_sum += lsm.gyroData.z;
            delay(5);
        }
        gyro_bias_x = gx_sum / 50.0f;
        gyro_bias_y = gy_sum / 50.0f;
        gyro_bias_z = gz_sum / 50.0f;
        DEBUG_PRINTLN("Done");
    }

    return success;
}

void read_imu_fast(int16_t* ax, int16_t* ay, int16_t* az, 
                   int16_t* gx, int16_t* gy, int16_t* gz) {
    if (!lsm_ready) {
        *ax=0; *ay=0; *az=0; *gx=0; *gy=0; *gz=0; return;
    }

    Wire.beginTransmission(LSM9DS1_XG_MEMS_ADDRESS);
    Wire.write(0x28); 
    Wire.endTransmission(false); 
    if (Wire.requestFrom((uint8_t)LSM9DS1_XG_MEMS_ADDRESS, (uint8_t)6) == 6) {
        uint8_t xlo = Wire.read(); uint8_t xhi = Wire.read();
        uint8_t ylo = Wire.read(); uint8_t yhi = Wire.read();
        uint8_t zlo = Wire.read(); uint8_t zhi = Wire.read();
        
        *ax = (int16_t)(xlo | (xhi << 8));
        *ay = (int16_t)(ylo | (yhi << 8));
        *az = (int16_t)(zlo | (zhi << 8));
    }

    Wire.beginTransmission(LSM9DS1_XG_MEMS_ADDRESS);
    Wire.write(0x18); 
    Wire.endTransmission(false); 
    if (Wire.requestFrom((uint8_t)LSM9DS1_XG_MEMS_ADDRESS, (uint8_t)6) == 6) {
        uint8_t xlo = Wire.read(); uint8_t xhi = Wire.read();
        uint8_t ylo = Wire.read(); uint8_t yhi = Wire.read();
        uint8_t zlo = Wire.read(); uint8_t zhi = Wire.read();
        
        *gx = (int16_t)(xlo | (xhi << 8));
        *gy = (int16_t)(ylo | (yhi << 8));
        *gz = (int16_t)(zlo | (zhi << 8));
    }
}

void read_imu_calibrated(float* ax, float* ay, float* az, 
                         float* gx, float* gy, float* gz) {
    
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    
    read_imu_fast(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // --- REMAPPING FOR VERTICAL MOUNTING ---
    // Rocket Z (Vertical) = Sensor Y
    // Rocket X = Sensor X
    // Rocket Y = Sensor Z (preserving right-hand rule)
    
    *az = (float)raw_ay * ACCEL_SCALE; // Vertical
    *ax = (float)raw_ax * ACCEL_SCALE;
    *ay = (float)raw_az * ACCEL_SCALE;

    *gz = ((float)raw_gy - gyro_bias_y) * GYRO_SCALE; // Vertical spin
    *gx = ((float)raw_gx - gyro_bias_x) * GYRO_SCALE;
    *gy = ((float)raw_gz - gyro_bias_z) * GYRO_SCALE;
}

void read_mag(int16_t* mx, int16_t* my, int16_t* mz) {
    if (!lsm_ready) { *mx=0; *my=0; *mz=0; return; }
    
    sensors_event_t a, m, g, t;
    lsm.getEvent(&a, &m, &g, &t);
    *mx = (int16_t)(m.magnetic.x * 100.0f);
    *my = (int16_t)(m.magnetic.y * 100.0f);
    *mz = (int16_t)(m.magnetic.z * 100.0f);
}

void write_i2c_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void sensors_set_low_power() {
    DEBUG_PRINTLN("[SENSORS] Entering Low Power...");

    if (lsm_ready) {
        write_i2c_reg(LSM9DS1_XG_MEMS_ADDRESS, LSM9DS1_REGISTER_CTRL_REG1_G, 0x38);
        write_i2c_reg(LSM9DS1_XG_MEMS_ADDRESS, LSM9DS1_REGISTER_CTRL_REG6_XL, 0x28);
        write_i2c_reg(LSM9DS1_MAG_ADDRESS, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00);
        write_i2c_reg(LSM9DS1_MAG_ADDRESS, 0x20, 0x00);
    }

    if (dps_ready) {
        dps.configurePressure(DPS310_1HZ, DPS310_4SAMPLES);
        dps.configureTemperature(DPS310_1HZ, DPS310_4SAMPLES);
    }
}

void sensors_set_flight_mode() {
    DEBUG_PRINTLN("[SENSORS] Entering Flight Mode...");
    
    if (lsm_ready) {
        write_i2c_reg(LSM9DS1_XG_MEMS_ADDRESS, LSM9DS1_REGISTER_CTRL_REG1_G, 0x98);
        write_i2c_reg(LSM9DS1_XG_MEMS_ADDRESS, LSM9DS1_REGISTER_CTRL_REG6_XL, 0x88);
        write_i2c_reg(LSM9DS1_MAG_ADDRESS, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00);
    }

    if (dps_ready) {
        dps.configurePressure(DPS310_64HZ, DPS310_8SAMPLES);
        dps.configureTemperature(DPS310_64HZ, DPS310_2SAMPLES);
    }
}

void read_dps368_data(float* pressure_pa, float* temp_celsius) {
    sensors_event_t temp_event, pressure_event;
    
    if (dps.pressureAvailable() && dps.temperatureAvailable()) {
        dps.getEvents(&temp_event, &pressure_event);
        
        *pressure_pa = pressure_event.pressure * 100.0f;
        *temp_celsius = temp_event.temperature;
    }
}
void continuity_init() {
    // Setup Drive Pin
    pinMode(CONT_DROGUE_ENABLE_PIN, OUTPUT);
    digitalWrite(CONT_DROGUE_ENABLE_PIN, LOW); // Default OFF
    
    pinMode(CONT_MAIN_ENABLE_PIN, OUTPUT);
    digitalWrite(CONT_MAIN_ENABLE_PIN, LOW);   // Default OFF

    // Setup ADC Pins
    adc_gpio_init(CONT_DROGUE_ADC_PIN_GPIO);
    adc_gpio_init(CONT_MAIN_ADC_PIN_GPIO);
    
    DEBUG_PRINTLN("[SENSORS] Continuity Monitor Initialized");
}

void read_continuity(uint8_t* drogue_status, uint8_t* main_status) {
    // --- CHECK DROGUE ---
    // 1. Turn ON Drive
    digitalWrite(CONT_DROGUE_ENABLE_PIN, HIGH);
    delayMicroseconds(200); // Stabilize
    
    // 2. Read ADC0
    adc_select_input(ADC_CHAN_DROGUE);
    uint16_t val_drogue = adc_read();
    
    // 3. Turn OFF Drive
    digitalWrite(CONT_DROGUE_ENABLE_PIN, LOW);

    // --- CHECK MAIN ---
    // 4. Turn ON Drive
    digitalWrite(CONT_MAIN_ENABLE_PIN, HIGH);
    delayMicroseconds(200); // Stabilize
    
    // 5. Read ADC1
    adc_select_input(ADC_CHAN_MAIN);
    uint16_t val_main = adc_read();
    
    // 6. Turn OFF Drive
    digitalWrite(CONT_MAIN_ENABLE_PIN, LOW);

    // --- DETERMINE STATUS ---
    *drogue_status = (val_drogue > CONT_THRESHOLD) ? 1 : 0;
    *main_status   = (val_main   > CONT_THRESHOLD) ? 1 : 0;
}