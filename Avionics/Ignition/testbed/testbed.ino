// Testbed for Arka Ignition System

// Libraries
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>

// --- MODE CONFIGURATION ---
#define MODE_MOTOR_TEST      // Comment this line out for "Ignition ONLY" mode (No SD/Logging/Data)

// Continuity Check Configuration
#define CHECK_CH_A           // Define this to check Channel A, comment out for Channel B
#ifdef CHECK_CH_A
  #define CONT_DRIVE_PIN     15  // Uses Igniter A pin to "pulse" for check
  #define CONT_ADC_PIN       8  // ADC linked to Channel A
#else
  #define CONT_DRIVE_PIN     16  // Uses Igniter B pin to "pulse" for check
  #define CONT_ADC_PIN       9  // ADC linked to Channel B
#endif

// Threshold for continuity check
#define CONT_THRESHOLD       300 

// Debugging Macros
#define DEBUG 1 
#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Macro Definitions
#define RYLR_BAUD            9600
#define LOG_INTERVAL_USEC    1000     
#define CAL_LOG_INTERVAL_MS  1000     
#define FILE_BASE_NAME       "/test"  
#define TELEM_INTERVAL_MS    350
#define COOLDOWN_MS          15000    
#define IGNITION_DURATION    3000     
#define RX_TIMEOUT_MS        5000

#define PIN_IGNITER_A        37
#define PIN_IGNITER_B        38
#define RYLR_RX              41
#define RYLR_TX              42

#define ADC_CH1              4
#define ADC_CH2              5
#define ADC_CH3              6
#define ADC_CH4              7

#define SD_CS                14
#define SD_MOSI              11
#define SD_CLK               12
#define SD_MISO              13

// Enums for logging and state management 
enum LogPhase { PHASE_CALIB = 0, PHASE_IGNITION_ON = 1, PHASE_IGNITION_OFF = 2, PHASE_ABORT = 3 };
enum SystemState { STATE_SAFE, STATE_ARM, STATE_LAUNCH, STATE_TRAP };

// Global Variables
HardwareSerial RYLR_SERIAL(2);
File dataFile;

// Timing Variables
hw_timer_t *timer = NULL;

// State Variables
SystemState currentState = STATE_SAFE;

// Communication Variables
String inputBuffer = "";
String response = "";

// Continuity Check Variables
bool requestSent = false;

// Sensor Data Variables
unsigned long requestSentTime = 0;
unsigned long lastCalibTime = 0;

// Continuity Status
bool continuityOK = false;

// Timer ISR Variables
volatile bool sampleTriggered = false;
volatile unsigned long triggeredTime = 0;
int16_t current_raw[4] = {0, 0, 0, 0};

// Timer ISR for periodic sampling occupying the CPU RAM for a short time
// So we set a flag and read sensors in the main loop to minimize ISR work
void IRAM_ATTR timerISR() {
  sampleTriggered = true;
  triggeredTime = micros();
}

// Pulse the continuity drive pin and read the ADC to check if the igniter circuit is intact
bool checkContinuity() {
  digitalWrite(CONT_DRIVE_PIN, HIGH);
  delayMicroseconds(200); 
  int adc = analogRead(CONT_ADC_PIN);
  digitalWrite(CONT_DRIVE_PIN, LOW);
  
  return (adc > CONT_THRESHOLD);
}

// Read incoming data from RYLR and store in response variable, returns true if a full line is received
void sendData(String data) {
  String tx = "AT+SEND=0," + String(data.length()) + "," + data;
  RYLR_SERIAL.println(tx);
  DEBUG_PRINT("[TX] "); DEBUG_PRINTLN(data);
}

// Reads the analog values from the ADC channels and stores them in the current_raw array
void readSensors() {
  current_raw[0] = analogRead(ADC_CH1);
  current_raw[1] = analogRead(ADC_CH2);
  current_raw[2] = analogRead(ADC_CH3);
  current_raw[3] = analogRead(ADC_CH4);
}

// Opens a new CSV file on the SD card for logging, with a unique name based on existing files. 
// Writes the header line to the file.
bool openFile() {
  #ifdef MODE_MOTOR_TEST
    char fileName[32];
    int i = 0;
    while (true) {
      sprintf(fileName, "%s_%03d.csv", FILE_BASE_NAME, i++);
      if (!SD.exists(fileName)) break;
    }
    dataFile = SD.open(fileName, FILE_WRITE);
    if (!dataFile) return false;
    dataFile.println("micros,phase,A0_Raw,A1_Raw,A2_Raw,A3_Raw,ContOK");
    dataFile.flush();
    return true;
  #else
    return true; // Simply return true if SD is disabled
  #endif
}

// Main sequence for handling the launch phase, including continuity check, igniter control, sensor reading, and telemetry transmission. 
// This function blocks until the launch sequence is complete or aborted. 
void executeLaunchSequence() {

  // Initial Continuity Check before firing
  if (!checkContinuity()) {
    sendData("ERR_NO_CONT");
    currentState = STATE_SAFE;
    return;
  }
  // Proceeds with ignition
  DEBUG_PRINTLN("--- FIRING ---");
  unsigned long startMicros = micros();
  unsigned long startMillis = millis();
  unsigned long lastTelemMs = 0;
  bool igniterIsOn = true;
  
  // Both pins HIGH to drive the igniter, timer starts to trigger periodic sampling in the ISR
  digitalWrite(PIN_IGNITER_A, HIGH);
  digitalWrite(PIN_IGNITER_B, HIGH);
  timerStart(timer); 

  // Main loop during launch phase, continues until ignition duration is exceeded or an abort command is received
  while (true) {
    if (RYLR_SERIAL.available()) {
      String cmd = RYLR_SERIAL.readStringUntil('\n');
      if (cmd.indexOf("AFE") >= 0 || cmd.indexOf("DISARM") >= 0) goto end_launch;
    }

    // Check if it's time to turn off the igniter after the specified duration
    if (sampleTriggered) {
      sampleTriggered = false;
      if (igniterIsOn && (micros() - startMicros >= (IGNITION_DURATION * 1000UL))) {
        digitalWrite(PIN_IGNITER_A, LOW);
        digitalWrite(PIN_IGNITER_B, LOW);
        igniterIsOn = false;
        
        // In Ignition ONLY mode, we can exit immediately after cutoff
        #ifndef MODE_MOTOR_TEST
           goto end_launch;
        #endif
      }

      // Inside executeLaunchSequence() -> while(true) -> if (sampleTriggered)
      #ifdef MODE_MOTOR_TEST
          readSensors();
          LogPhase phase = igniterIsOn ? PHASE_IGNITION_ON : PHASE_IGNITION_OFF;
          if (dataFile) {
              dataFile.printf("%lu,%d,%d,%d,%d,%d,%d\n", triggeredTime, phase, current_raw[0], current_raw[1], current_raw[2], current_raw[3], continuityOK);
          }
      
          // --- UPDATED TELEMETRY WITH TIMESTAMP ---
          if (millis() - lastTelemMs >= TELEM_INTERVAL_MS) {
            char telemBuf[128];
            LogPhase phase = igniterIsOn ? PHASE_IGNITION_ON : PHASE_IGNITION_OFF;

              // Format matches your requirement: T:[micros],C:[cont],M:[val0],[val1],[val2],[val3]
            snprintf(telemBuf, sizeof(telemBuf), "T:%lu,C:%d,M:%d,%d,%d,%d", 
                      triggeredTime, 
                      continuityOK, 
                      current_raw[0], 
                      current_raw[1], 
                      current_raw[2], 
                      current_raw[3]);
            sendData(telemBuf);
            lastTelemMs = millis();
          }
      #endif
    }
    // Yields to allow other processes to run, preventing the loop from blocking completely. 
    yield(); 
  }

// Common exit point for ending the launch sequence, ensures igniter is turned off, timer is stopped, file is closed, and state is set to trap.
end_launch:
  timerStop(timer); 
  digitalWrite(PIN_IGNITER_A, LOW);
  digitalWrite(PIN_IGNITER_B, LOW);
  #ifdef MODE_MOTOR_TEST
    if (dataFile) { dataFile.flush(); dataFile.close(); }
  #endif
  sendData("SAFE");
  // After a launch or abort, we enter a trap state to prevent further actions until reset, ensuring safety and data integrity.
  currentState = STATE_TRAP;
}

// Setup function
void setup() {
  #if DEBUG
    Serial.begin(115200);
  #endif
  RYLR_SERIAL.begin(RYLR_BAUD, SERIAL_8N1, RYLR_RX, RYLR_TX); // Initialize RYLR serial communication with specified baud rate and pins
  pinMode(PIN_IGNITER_A, OUTPUT);
  pinMode(PIN_IGNITER_B, OUTPUT);
  
  digitalWrite(PIN_IGNITER_A, LOW); 
  digitalWrite(PIN_IGNITER_B, LOW);

  analogReadResolution(12); // Set ADC resolution to 12 bits for finer granularity in sensor readings
  analogSetAttenuation(ADC_11db); // Set attenuation to 11dB for a wider input voltage range, allowing for better measurement of higher voltages that may be present in the igniter circuit.

  delay(1000);
  RYLR_SERIAL.println("AT+PARAMETER=9,7,1,12"); // Configure RYLR module with specific parameters
  
  // Initialize SD card communication using SPI, check if the SD card is present and can be initialized. 
  // If not, enter trap state and send error message. If successful, set state to safe and send ready message.
  #ifdef MODE_MOTOR_TEST
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
      currentState = STATE_TRAP;
      sendData("ERR_SD_TRAP");
    } else {
      currentState = STATE_SAFE;
      sendData("READY");
    }
  #else
    currentState = STATE_SAFE;
    sendData("READY");
  #endif

  // Set up the hardware timer for periodic sampling during the launch phase, with an interval defined by LOG_INTERVAL_USEC.
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &timerISR);
  timerAlarm(timer, 1000000 / LOG_INTERVAL_USEC, true, 0); 
}

bool receiveData() {
  while (RYLR_SERIAL.available()) {
    char c = RYLR_SERIAL.read();
    if (c == '\n') {
      inputBuffer.trim();
      response = inputBuffer;
      inputBuffer = "";
      return true;
    } else {
      inputBuffer += c;
    }
  }
  return false;
}

void loop() {
  switch (currentState) {
    case STATE_SAFE:
      // In the safe state, we wait for an "ARM" command from the RYLR module to transition to the armed state.
      if (!requestSent) { 
        sendData("REQ_ARM"); requestSent = true; requestSentTime = millis();
      }

      // Check for incoming data from RYLR, if we receive an "ARM" command, we attempt to open a new file for logging and transition to the armed state.
      if (receiveData()) {
        if (response.indexOf("ARM") >= 0) { 
          sendData("ARMED");
          if (openFile()) currentState = STATE_ARM;
          else { sendData("ERR_FILE"); currentState = STATE_TRAP; }
          requestSent = false;
        }
      }

      // If we have sent a request but haven't received a response within the timeout period, we reset the requestSent flag to allow sending another request.
      if (requestSent && (millis() - requestSentTime > RX_TIMEOUT_MS)) requestSent = false;
      delay(10);
      break;

    case STATE_ARM:
      // In the armed state, we perform periodic sensor readings and logging, and wait for a "LAUNCH" command to transition to the launch sequence.
      if (millis() - lastCalibTime >= CAL_LOG_INTERVAL_MS) {
        #ifdef MODE_MOTOR_TEST
          readSensors();
          if (dataFile) {
            dataFile.printf("%lu,%d,%d,%d,%d,%d\n", micros(), PHASE_CALIB, current_raw[0], current_raw[1], current_raw[2], current_raw[3]);
            dataFile.flush();
          }
        #endif
        lastCalibTime = millis();
      }

      // We send a "REQ_LAUNCH" command to the RYLR module to indicate that we are ready for launch, and wait for a "LAUNCH" command in response. If we receive "LAUNCH", we transition to the launch sequence. If we receive "DISARM", we transition back to safe state and close the file.
      if (!requestSent) { sendData("REQ_LAUNCH"); requestSent = true; requestSentTime = millis(); }
      if (receiveData()) {
        if (response.indexOf("LAUNCH") >= 0) {
          sendData("LAUNCHING");
          currentState = STATE_LAUNCH; 
          // We call the executeLaunchSequence function which will handle the entire launch process, including continuity checks, igniter control, sensor reading, logging, and telemetry transmission. This function will block until the launch sequence is complete or aborted.
          executeLaunchSequence(); 
          currentState = STATE_TRAP;
        } else if (response.indexOf("DISARM") >= 0) {
          sendData("SAFE");
          currentState = STATE_SAFE;
          #ifdef MODE_MOTOR_TEST
            if(dataFile) dataFile.close();
          #endif
          requestSent = false;
        }
      }
      // Similar to the safe state, if we have sent a request but haven't received a response within the timeout period, we reset the requestSent flag to allow sending another request.
      if (requestSent && (millis() - requestSentTime > RX_TIMEOUT_MS)) requestSent = false;
      delay(10);
      break;

    case STATE_TRAP:
      // In the trap state, we do nothing and wait for a reset. 
      // This state is entered after a launch sequence or if a critical error occurs (like SD card failure), 
      // to prevent any further actions until the system is reset.
      delay(1000);
      break;
  }
}