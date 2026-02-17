#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Watchdog_t4.h>

// Pin Definitions
const uint8_t SERVO_X_PIN = 2;
const uint8_t SERVO_Y_PIN = 3;
const uint8_t BUZZER_PIN = 7;
const uint8_t TOUCH_CS = 10;
const uint8_t TOUCH_PEN = 8;
const uint8_t IMU_RST = 9;

// Physical Dimensions
const float PLATE_WIDTH_MM = 76.7f;
const float PLATE_HEIGHT_MM = 63.65f;
const float PLATE_CENTER_X_MM = PLATE_WIDTH_MM / 2.0f;
const float PLATE_CENTER_Y_MM = PLATE_HEIGHT_MM / 2.0f;

// Touchscreen Calibration
const int TOUCH_X_MIN = 200;
const int TOUCH_X_MAX = 3900;
const int TOUCH_Y_MIN = 260;
const int TOUCH_Y_MAX = 3870;
const int SCREEN_WIDTH = 320;
const int SCREEN_HEIGHT = 240;
const float TOUCH_FILTER_ALPHA = 0.7f;

// Servo Mechanical Mapping
const float SERVO_X_CENTER_OFFSET = 0.0f;
const float SERVO_Y_CENTER_OFFSET = 0.0f;
const float SERVO_X_LIMIT = 28.0f;
const float SERVO_Y_LIMIT = 23.0f;
const float PLATE_X_LIMIT = 12.0f;
const float PLATE_Y_LIMIT = 10.0f;
const int GAIN_TABLE_SIZE = 7;
float servoX_table[GAIN_TABLE_SIZE] = {-28, -20, -10,  0, 10, 20, 28};
float plateX_table[GAIN_TABLE_SIZE] = {-12, -8.5, -4.3, 0, 4.3, 8.5, 12};
float servoY_table[GAIN_TABLE_SIZE] = {-23, -15, -8,  0,  8, 15, 23};
float plateY_table[GAIN_TABLE_SIZE] = {-10, -6.5, -3.5, 0, 3.5, 6.5, 10};
const int SERVO_CENTER = 1500;
const int SERVO_MIN = 1000;
const int SERVO_MAX = 2000;
const float SERVO_US_PER_DEGREE = 5.556f;
const float SERVO_DEADBAND = 1.5f;
const float SERVO_MAX_SLEW_RATE = 300.0f;

// Control Parameters
const float KP_X = 0.4f;
const float KP_Y = 0.4f;
const float KI_X = 0.08f;
const float KI_Y = 0.08f;
const float KD_X = 0.15f;
const float KD_Y = 0.15f;
const float INTEGRAL_LIMIT_X = 25.0f;
const float INTEGRAL_LIMIT_Y = 25.0f;
const float DEADZONE_MM = 2.0f;
const float TARGET_X_MM = PLATE_CENTER_X_MM;
const float TARGET_Y_MM = PLATE_CENTER_Y_MM;

// Safety Parameters
const unsigned long TOUCH_WATCHDOG_MS = 500;
const float INSTABILITY_ERROR_THRESHOLD_MM = 40.0f;
const unsigned long INSTABILITY_TIMEOUT_MS = 2000;
const float SOFT_RECENTER_RATE = 5.0f;
const float MAX_VELOCITY_JUMP_MM = 30.0f;
const unsigned long UART_RX_TIMEOUT_MS = 200;
const unsigned long SERVO_STUCK_THRESHOLD = 100;

// Timing Constants
const unsigned long CONTROL_INTERVAL_US = 10000;
const unsigned long UART_INTERVAL_MS = 20;
const unsigned long IMU_INTERVAL_MS = 50;
const unsigned long DEBUG_INTERVAL_MS = 500;
const unsigned long TOUCH_TIMEOUT_MS = 100;
const float CONTROL_DT_SECONDS = CONTROL_INTERVAL_US / 1000000.0f;

// UART Protocol
const uint8_t UART_STX = 0x02;
const uint8_t UART_ETX = 0x03;
const unsigned long UART_BAUD = 115200;

// Global Variables
Servo servoX;
Servo servoY;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imuAvailable = false;
float imuRoll = 0.0f;
float imuPitch = 0.0f;
float imuYaw = 0.0f;
int touchX_raw = 0;
int touchY_raw = 0;
float touchX_filtered = 0.0f;
float touchY_filtered = 0.0f;
bool touchValid = false;
bool ballDetected = false;
unsigned long lastTouchTime = 0;
unsigned long lastValidTouchTime = 0;
float ballX_mm = PLATE_CENTER_X_MM;
float ballY_mm = PLATE_CENTER_Y_MM;
float ballVX_mms = 0.0f;
float ballVY_mms = 0.0f;
float prevBallX_mm = PLATE_CENTER_X_MM;
float prevBallY_mm = PLATE_CENTER_Y_MM;
unsigned long prevVelTime = 0;
float integralX = 0.0f;
float integralY = 0.0f;
float plateAngleX_cmd = 0.0f;
float plateAngleY_cmd = 0.0f;
float servoAngleX_actual = 0.0f;
float servoAngleY_actual = 0.0f;
unsigned long lastControlTime = 0;
unsigned long lastUartTime = 0;
unsigned long lastImuTime = 0;
unsigned long lastDebugTime = 0;
unsigned long maxLoopTime_us = 0;
bool systemReady = false;
bool emergencyStop = false;
unsigned long instabilityStartTime = 0;
unsigned long lastUartRxTime = 0;
int lastServoX_us = SERVO_CENTER;
int lastServoY_us = SERVO_CENTER;
unsigned long servoStuckCount = 0;
unsigned long lastSaturationWarning = 0;
SPISettings touchSPISettings(2000000, MSBFIRST, SPI_MODE0);
WDT_timings_t wdtConfig;
WDT_T4<WDT1> wdt;

// Watchdog Callback
void watchdogTimeout() {
  servoX.writeMicroseconds(SERVO_CENTER);
  servoY.writeMicroseconds(SERVO_CENTER);
  digitalWrite(BUZZER_PIN, HIGH);
}

// Helper Functions
float interpolate(float x, float* x_table, float* y_table, int size) {
  if (x <= x_table[0]) return y_table[0];
  if (x >= x_table[size-1]) return y_table[size-1];
  for (int i = 0; i < size - 1; i++) {
    if (x >= x_table[i] && x <= x_table[i+1]) {
      float denominator = x_table[i+1] - x_table[i];
      if (abs(denominator) < 0.001f) {
        return y_table[i];
      }
      float t = (x - x_table[i]) / denominator;
      return y_table[i] + t * (y_table[i+1] - y_table[i]);
    }
  }
  return 0.0f;
}

float plateToServoX(float plate_angle) {
  return interpolate(plate_angle, plateX_table, servoX_table, GAIN_TABLE_SIZE);
}

float plateToServoY(float plate_angle) {
  return interpolate(plate_angle, plateY_table, servoY_table, GAIN_TABLE_SIZE);
}

float applyDeadband(float angle, float deadband) {
  if (abs(angle) < deadband) return 0.0f;
  return angle;
}

uint16_t crc16_ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
      crc &= 0xFFFF;
    }
  }
  return crc;
}

void buzz(int duration_ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// Startup Centering Sequence
void startupCenteringSequence() {
  Serial.println(F("Executing startup centering sequence..."));
  const int STEPS = 50;
  const int STEP_DELAY_MS = 20;
  for (int i = 0; i <= STEPS; i++) {
    wdt.feed();
    float progress = (float)i / STEPS;
    float angleX = servoAngleX_actual * (1.0f - progress);
    float angleY = servoAngleY_actual * (1.0f - progress);
    int servoX_us = SERVO_CENTER + (int)round(angleX * SERVO_US_PER_DEGREE);
    int servoY_us = SERVO_CENTER + (int)round(angleY * SERVO_US_PER_DEGREE);
    servoX_us = constrain(servoX_us, SERVO_MIN, SERVO_MAX);
    servoY_us = constrain(servoY_us, SERVO_MIN, SERVO_MAX);
    servoX.writeMicroseconds(servoX_us);
    servoY.writeMicroseconds(servoY_us);
    delay(STEP_DELAY_MS);
  }
  servoAngleX_actual = 0.0f;
  servoAngleY_actual = 0.0f;
  plateAngleX_cmd = 0.0f;
  plateAngleY_cmd = 0.0f;
  Serial.println(F("✓ Startup centering complete"));
  buzz(200);
}

// Touchscreen SPI Functions
uint16_t readTouch(uint8_t cmd) {
  uint8_t msb, lsb;
  SPI.beginTransaction(touchSPISettings);
  digitalWrite(TOUCH_CS, LOW);
  SPI.transfer(cmd);
  msb = SPI.transfer(0x00);
  lsb = SPI.transfer(0x00);
  digitalWrite(TOUCH_CS, HIGH);
  SPI.endTransaction();
  return ((msb << 8) | lsb) >> 3;
}

bool isTouched() {
  return digitalRead(TOUCH_PEN) == LOW;
}

void readTouchPosition() {
  if (!isTouched()) {
    touchValid = false;
    ballDetected = false;
    return;
  }
  touchX_raw = readTouch(0xD0);
  touchY_raw = readTouch(0x90);
  int z1 = readTouch(0xB0);
  int z2 = readTouch(0xC0);
  int pressure = z1 - z2;
  if (pressure < 50 || pressure > 4000) {
    touchValid = false;
    ballDetected = false;
    return;
  }
  if (touchX_raw < TOUCH_X_MIN || touchX_raw > TOUCH_X_MAX ||
      touchY_raw < TOUCH_Y_MIN || touchY_raw > TOUCH_Y_MAX) {
    touchValid = false;
    ballDetected = false;
    return;
  }
  int x_px = map(touchX_raw, TOUCH_X_MIN, TOUCH_X_MAX, 0, SCREEN_WIDTH);
  int y_px = map(touchY_raw, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, SCREEN_HEIGHT);
  if (!touchValid) {
    touchX_filtered = x_px;
    touchY_filtered = y_px;
  } else {
    touchX_filtered = TOUCH_FILTER_ALPHA * x_px + (1.0f - TOUCH_FILTER_ALPHA) * touchX_filtered;
    touchY_filtered = TOUCH_FILTER_ALPHA * y_px + (1.0f - TOUCH_FILTER_ALPHA) * touchY_filtered;
  }
  touchValid = true;
  ballDetected = true;
  lastTouchTime = millis();
  lastValidTouchTime = millis();
}

void convertTouchToMM() {
  float scale_x = PLATE_WIDTH_MM / SCREEN_WIDTH;
  float scale_y = PLATE_HEIGHT_MM / SCREEN_HEIGHT;
  float centerX_px = SCREEN_WIDTH / 2.0f;
  float centerY_px = SCREEN_HEIGHT / 2.0f;
  ballX_mm = PLATE_CENTER_X_MM + (touchX_filtered - centerX_px) * scale_x;
  ballY_mm = PLATE_CENTER_Y_MM + (touchY_filtered - centerY_px) * scale_y;
  ballX_mm = constrain(ballX_mm, 0, PLATE_WIDTH_MM);
  ballY_mm = constrain(ballY_mm, 0, PLATE_HEIGHT_MM);
}

// Velocity Computation
void computeVelocity() {
  unsigned long now = micros();
  if (prevVelTime > 0) {
    unsigned long elapsed_us = (unsigned long)(now - prevVelTime);
    float dt = elapsed_us / 1000000.0f;
    if (dt > 0.001f && dt < 0.1f) {
      float dx = ballX_mm - prevBallX_mm;
      float dy = ballY_mm - prevBallY_mm;
      float displacement = sqrt(dx*dx + dy*dy);
      if (displacement < MAX_VELOCITY_JUMP_MM) {
        ballVX_mms = dx / dt;
        ballVY_mms = dy / dt;
      } else {
        ballVX_mms = 0.0f;
        ballVY_mms = 0.0f;
      }
    }
  }
  prevBallX_mm = ballX_mm;
  prevBallY_mm = ballY_mm;
  prevVelTime = now;
}

// Touch Dropout Recovery
void handleTouchDropout() {
  integralX = 0.0f;
  integralY = 0.0f;
  ballVX_mms = 0.0f;
  ballVY_mms = 0.0f;
  prevVelTime = 0;
  float max_step = SOFT_RECENTER_RATE * CONTROL_DT_SECONDS;
  if (abs(plateAngleX_cmd) > 0.1f) {
    float step = constrain(-plateAngleX_cmd, -max_step, max_step);
    plateAngleX_cmd += step;
  } else {
    plateAngleX_cmd = 0.0f;
  }
  if (abs(plateAngleY_cmd) > 0.1f) {
    float step = constrain(-plateAngleY_cmd, -max_step, max_step);
    plateAngleY_cmd += step;
  } else {
    plateAngleY_cmd = 0.0f;
  }
}

// Safety Watchdog
void checkSafetyWatchdog() {
  unsigned long now = millis();
  if ((unsigned long)(now - lastValidTouchTime) > TOUCH_WATCHDOG_MS) {
    if (!emergencyStop) {
      Serial.println(F("⚠ EMERGENCY STOP: Touch watchdog timeout!"));
      emergencyStop = true;
      buzz(1000);
    }
  }
  if ((unsigned long)(now - lastUartRxTime) > UART_RX_TIMEOUT_MS) {
    if (!emergencyStop) {
      Serial.println(F("⚠ EMERGENCY STOP: UART timeout!"));
      Serial.println(F("  Pi may have crashed or disconnected"));
      emergencyStop = true;
      buzz(1000);
    }
  }
  if (ballDetected) {
    float errorX = TARGET_X_MM - ballX_mm;
    float errorY = TARGET_Y_MM - ballY_mm;
    float errorDist = sqrt(errorX * errorX + errorY * errorY);
    if (errorDist > INSTABILITY_ERROR_THRESHOLD_MM) {
      if (instabilityStartTime == 0) {
        instabilityStartTime = now;
      } else if ((unsigned long)(now - instabilityStartTime) > INSTABILITY_TIMEOUT_MS) {
        if (!emergencyStop) {
          Serial.println(F("⚠ EMERGENCY STOP: System unstable!"));
          Serial.print(F("  Error: "));
          Serial.print(errorDist, 1);
          Serial.println(F(" mm"));
          emergencyStop = true;
          buzz(1000);
        }
      }
    } else {
      instabilityStartTime = 0;
    }
  }
  if (emergencyStop) {
    plateAngleX_cmd = 0.0f;
    plateAngleY_cmd = 0.0f;
    integralX = 0.0f;
    integralY = 0.0f;
    ballVX_mms = 0.0f;
    ballVY_mms = 0.0f;
    prevVelTime = 0;
  }
}

// PI(D) Control
void computeControl() {
  if (emergencyStop) {
    return;
  }
  float errorX_mm = TARGET_X_MM - ballX_mm;
  float errorY_mm = TARGET_Y_MM - ballY_mm;
  float errorDist = sqrt(errorX_mm * errorX_mm + errorY_mm * errorY_mm);
  if (errorDist < DEADZONE_MM) {
    return;
  }
  bool saturatedX = (abs(plateAngleX_cmd) >= PLATE_X_LIMIT * 0.95f);
  bool saturatedY = (abs(plateAngleY_cmd) >= PLATE_Y_LIMIT * 0.95f);
  if (saturatedX && abs(errorX_mm) > DEADZONE_MM) {
    if ((unsigned long)(millis() - lastSaturationWarning) > 5000) {
      Serial.println(F("⚠ X-axis saturated - integrator frozen"));
      lastSaturationWarning = millis();
    }
  }
  if (!saturatedX) {
    integralX += errorX_mm * CONTROL_DT_SECONDS;
    integralX = constrain(integralX, -INTEGRAL_LIMIT_X, INTEGRAL_LIMIT_X);
  }
  if (!saturatedY) {
    integralY += errorY_mm * CONTROL_DT_SECONDS;
    integralY = constrain(integralY, -INTEGRAL_LIMIT_Y, INTEGRAL_LIMIT_Y);
  }
  plateAngleX_cmd = KP_X * errorX_mm + KI_X * integralX - KD_X * ballVX_mms;
  plateAngleY_cmd = KP_Y * errorY_mm + KI_Y * integralY - KD_Y * ballVY_mms;
  plateAngleX_cmd = constrain(plateAngleX_cmd, -PLATE_X_LIMIT, PLATE_X_LIMIT);
  plateAngleY_cmd = constrain(plateAngleY_cmd, -PLATE_Y_LIMIT, PLATE_Y_LIMIT);
}

// Servo Update with Slew Limit and Stuck Detection
void updateServosWithSlewLimit() {
  float servoX_cmd = plateToServoX(plateAngleX_cmd);
  float servoY_cmd = plateToServoY(plateAngleY_cmd);
  servoX_cmd = applyDeadband(servoX_cmd, SERVO_DEADBAND);
  servoY_cmd = applyDeadband(servoY_cmd, SERVO_DEADBAND);
  servoX_cmd = constrain(servoX_cmd, -SERVO_X_LIMIT, SERVO_X_LIMIT);
  servoY_cmd = constrain(servoY_cmd, -SERVO_Y_LIMIT, SERVO_Y_LIMIT);
  float max_step = SERVO_MAX_SLEW_RATE * CONTROL_DT_SECONDS;
  float deltaX = servoX_cmd - servoAngleX_actual;
  float deltaY = servoY_cmd - servoAngleY_actual;
  servoAngleX_actual += constrain(deltaX, -max_step, max_step);
  servoAngleY_actual += constrain(deltaY, -max_step, max_step);
  servoAngleX_actual = constrain(servoAngleX_actual, -SERVO_X_LIMIT, SERVO_X_LIMIT);
  servoAngleY_actual = constrain(servoAngleY_actual, -SERVO_Y_LIMIT, SERVO_Y_LIMIT);
  int servoX_us = SERVO_CENTER + (int)round((servoAngleX_actual + SERVO_X_CENTER_OFFSET) * SERVO_US_PER_DEGREE);
  int servoY_us = SERVO_CENTER + (int)round((servoAngleY_actual + SERVO_Y_CENTER_OFFSET) * SERVO_US_PER_DEGREE);
  servoX_us = constrain(servoX_us, SERVO_MIN, SERVO_MAX);
  servoY_us = constrain(servoY_us, SERVO_MIN, SERVO_MAX);
  if (abs(servoX_cmd - servoAngleX_actual) > 1.0f) {
    if (abs(servoX_us - lastServoX_us) < 1) {
      servoStuckCount++;
      if (servoStuckCount > SERVO_STUCK_THRESHOLD) {
        Serial.println(F("⚠ WARNING: Servo may be stuck or disconnected!"));
        servoStuckCount = 0;
      }
    } else {
      servoStuckCount = 0;
    }
  }
  lastServoX_us = servoX_us;
  lastServoY_us = servoY_us;
  servoX.writeMicroseconds(servoX_us);
  servoY.writeMicroseconds(servoY_us);
}

// IMU Reading
void readIMU() {
  sensors_event_t event;
  bno.getEvent(&event);
  imuRoll = event.orientation.y;
  imuPitch = event.orientation.z;
  imuYaw = event.orientation.x;
  if (isnan(imuRoll)) imuRoll = 0.0f;
  if (isnan(imuPitch)) imuPitch = 0.0f;
  if (isnan(imuYaw)) imuYaw = 0.0f;
}

// UART Telemetry
void sendUartData() {
  char payload[128];
  int len;
  if (imuAvailable) {
    len = snprintf(payload, sizeof(payload),
                   "%lu,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                   micros(),
                   ballX_mm, ballY_mm,
                   ballDetected ? 1 : 0,
                   emergencyStop ? 1 : 0,
                   plateAngleX_cmd, plateAngleY_cmd,
                   servoAngleX_actual, servoAngleY_actual,
                   imuRoll, imuPitch, imuYaw);
  } else {
    len = snprintf(payload, sizeof(payload),
                   "%lu,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,,,",
                   micros(),
                   ballX_mm, ballY_mm,
                   ballDetected ? 1 : 0,
                   emergencyStop ? 1 : 0,
                   plateAngleX_cmd, plateAngleY_cmd,
                   servoAngleX_actual, servoAngleY_actual);
  }
  if (len < 0 || len >= sizeof(payload)) {
    return;
  }
  uint16_t crc = crc16_ccitt((uint8_t*)payload, len);
  Serial1.write(UART_STX);
  Serial1.write((uint8_t)len);
  Serial1.write((uint8_t*)payload, len);
  Serial1.write((uint8_t)(crc >> 8));
  Serial1.write((uint8_t)(crc & 0xFF));
  Serial1.write(UART_ETX);
  lastUartRxTime = millis();
}

// Debug Output
void printDebug() {
  if (emergencyStop) {
    Serial.println(F("🛑 EMERGENCY STOP ACTIVE"));
    return;
  }
  Serial.print(F("Ball: "));
  if (ballDetected) {
    Serial.print(F("("));
    Serial.print(ballX_mm, 1);
    Serial.print(F(", "));
    Serial.print(ballY_mm, 1);
    Serial.print(F(") mm | Err: "));
    float error = sqrt(pow(TARGET_X_MM - ballX_mm, 2) + pow(TARGET_Y_MM - ballY_mm, 2));
    Serial.print(error, 1);
    Serial.print(F(" mm"));
  } else {
    Serial.print(F("LOST"));
  }
  Serial.print(F(" | Cmd: ("));
  Serial.print(plateAngleX_cmd, 1);
  Serial.print(F("°, "));
  Serial.print(plateAngleY_cmd, 1);
  Serial.print(F("°) | Act: ("));
  Serial.print(servoAngleX_actual, 1);
  Serial.print(F("°, "));
  Serial.print(servoAngleY_actual, 1);
  Serial.print(F("°)"));
  if (imuAvailable) {
    Serial.print(F(" | IMU: "));
    Serial.print(imuPitch, 1);
    Serial.print(F("°/"));
    Serial.print(imuRoll, 1);
    Serial.print(F("°"));
  }
  Serial.print(F(" | Max: "));
  Serial.print(maxLoopTime_us);
  Serial.println(F(" µs"));
  maxLoopTime_us = 0;
}

// Setup
void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD);
  delay(1000);
  Serial.println(F("==================================="));
  Serial.println(F("Ball Balancer - PRODUCTION HARDENED"));
  Serial.println(F("==================================="));
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);
  pinMode(TOUCH_PEN, INPUT);
  pinMode(IMU_RST, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  digitalWrite(IMU_RST, HIGH);
  SPI.begin();
  Wire.begin();
  wdtConfig.window = 0;
  wdtConfig.timeout = 500;
  wdtConfig.callback = watchdogTimeout;
  wdt.begin(wdtConfig);
  Serial.println(F("✓ Hardware watchdog enabled (500ms)"));
  Serial.println(F("Initializing IMU..."));
  if (bno.begin()) {
    bno.setExtCrystalUse(true);
    imuAvailable = true;
    Serial.println(F("✓ IMU ready"));
  } else {
    Serial.println(F("⚠ IMU not found"));
  }
  Serial.println(F("Attaching servos..."));
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  Serial.println(F("✓ Servos attached"));
  startupCenteringSequence();
  Serial.println(F("Safety Features:"));
  Serial.println(F("  • Hardware watchdog (500ms)"));
  Serial.println(F("  • UART timeout (200ms)"));
  Serial.println(F("  • Servo stuck detection (1s)"));
  Serial.println(F("  • Anti-windup logging"));
  Serial.println();
  buzz(100);
  delay(100);
  buzz(100);
  systemReady = true;
  lastControlTime = micros();
  lastUartTime = millis();
  lastImuTime = millis();
  lastDebugTime = millis();
  lastValidTouchTime = millis();
  lastUartRxTime = millis();
}

// Main Loop
void loop() {
  wdt.feed();
  unsigned long loopStart = micros();
  unsigned long now_us = micros();
  unsigned long now_ms = millis();
  readTouchPosition();
  if ((unsigned long)(now_us - lastControlTime) >= CONTROL_INTERVAL_US) {
    lastControlTime += CONTROL_INTERVAL_US;
    if (touchValid && ballDetected) {
      convertTouchToMM();
      computeVelocity();
      computeControl();
    } else {
      handleTouchDropout();
    }
    checkSafetyWatchdog();
    updateServosWithSlewLimit();
  }
  if ((unsigned long)(now_ms - lastUartTime) >= UART_INTERVAL_MS) {
    lastUartTime = now_ms;
    sendUartData();
  }
  if (imuAvailable && (unsigned long)(now_ms - lastImuTime) >= IMU_INTERVAL_MS) {
    lastImuTime = now_ms;
    readIMU();
  }
  if ((unsigned long)(now_ms - lastDebugTime) >= DEBUG_INTERVAL_MS) {
    lastDebugTime = now_ms;
    printDebug();
  }
  unsigned long loopTime = micros() - loopStart;
  if (loopTime > maxLoopTime_us) {
    maxLoopTime_us = loopTime;
  }
}