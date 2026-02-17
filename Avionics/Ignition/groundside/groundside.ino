// Groundside Control Station for Arka Ignition System

// Libraries
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>

// --- MODE CONFIGURATION ---
#define MODE_MOTOR_TEST      // Comment this out if Groundside has no SD card

// Macro Definitions
#define RYLR_BAUD            9600
#define FILE_BASE_NAME       "/GND_LOG"

#define ARM_SWITCH_PIN       4
#define LAUNCH_SWITCH_PIN    5
#define SAFE_SWITCH_PIN      6

#define RYLR_RX              41
#define RYLR_TX              42

#define SD_CS                14
#define SD_MOSI              11
#define SD_CLK               12
#define SD_MISO              13

// Global Variables
HardwareSerial RYLR_SERIAL(2);
File dataFile;

// State Variables
bool lastArmSwitchState = false;
bool lastLaunchSwitchState = false;
bool lastSafeSwitchState = false;

// Input Buffer for RYLR Serial Data
String inputBuffer = "";

// Continuity Status
bool rocketContOK = false;

// Function to open a new log file on the SD card
bool openLogFile() {
  #ifdef MODE_MOTOR_TEST
    char fileName[32];
    int i = 0;
    while (true) {
      sprintf(fileName, "%s_%03d.csv", FILE_BASE_NAME, i++);
      if (!SD.exists(fileName)) break;
    }
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("millis,event,raw_packet");
      dataFile.flush();
      Serial.print("Ground Log Started: "); Serial.println(fileName);
      return true;
    }
    return false;
  #else
    return true; 
  #endif
}

// Function to log events to the SD card
void logToSD(String event, String packet) {
  #ifdef MODE_MOTOR_TEST
    if (dataFile) {
      dataFile.printf("%lu,%s,%s\n", millis(), event.c_str(), packet.c_str());
      dataFile.flush();
    }
  #endif
}

// Function to check the state of the switches and send commands accordingly
void checkSwitches() {
  bool currentArmState = digitalRead(ARM_SWITCH_PIN);
  bool currentLaunchState = digitalRead(LAUNCH_SWITCH_PIN);
  bool currentSafeState = digitalRead(SAFE_SWITCH_PIN);

  if (currentSafeState == true && lastSafeSwitchState == false) {
    String cmd = "DISARM";
    String rylrCmd = "AT+SEND=0," + String(cmd.length()) + "," + cmd + "\r\n";
    Serial.print("!!! SAFE SWITCH ON -> "); Serial.print(rylrCmd);
    RYLR_SERIAL.print(rylrCmd);
    logToSD("TX_SAFE_SWITCH", cmd);
    delay(50);
  }

  if (!currentSafeState) {
    if (currentArmState == true && lastArmSwitchState == false) {
      String cmd = "ARM";
      String rylrCmd = "AT+SEND=0," + String(cmd.length()) + "," + cmd + "\r\n";
      Serial.print("SWITCH CMD: ARM -> "); Serial.print(rylrCmd);
      RYLR_SERIAL.print(rylrCmd);
      logToSD("TX_ARM", cmd);
      delay(50);
    }

    if (currentArmState == false && lastArmSwitchState == true) {
      String cmd = "DISARM";
      String rylrCmd = "AT+SEND=0," + String(cmd.length()) + "," + cmd + "\r\n";
      Serial.print("SWITCH CMD: DISARM -> "); Serial.print(rylrCmd);
      RYLR_SERIAL.print(rylrCmd);
      logToSD("TX_DISARM", cmd);
      delay(50);
    }

    if (currentLaunchState == true && lastLaunchSwitchState == false && currentArmState == true) {
      if (rocketContOK) {
        String cmd = "LAUNCH";
        String rylrCmd = "AT+SEND=0," + String(cmd.length()) + "," + cmd + "\r\n";
        Serial.print("SWITCH CMD: LAUNCH -> "); Serial.print(rylrCmd);
        RYLR_SERIAL.print(rylrCmd);
        logToSD("TX_LAUNCH", cmd);
      } else {
        Serial.println("❌ BLOCKED: NO CONTINUITY");
      }
      delay(50);
    }
  }

  lastArmSwitchState = currentArmState;
  lastLaunchSwitchState = currentLaunchState;
  lastSafeSwitchState = currentSafeState;
}

void setup() {
  Serial.begin(115200);
  RYLR_SERIAL.begin(RYLR_BAUD, SERIAL_8N1, RYLR_RX, RYLR_TX); 

  pinMode(ARM_SWITCH_PIN, INPUT);
  pinMode(LAUNCH_SWITCH_PIN, INPUT);
  pinMode(SAFE_SWITCH_PIN, INPUT); 

  #ifdef MODE_MOTOR_TEST
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS)) openLogFile();
  #endif

  delay(1000);
  RYLR_SERIAL.println("AT+PARAMETER=9,7,1,12"); 
  Serial.println("Ground Station Ready. Monitoring Rocket...");
}

void loop() {
  while (RYLR_SERIAL.available()) {
    char c = RYLR_SERIAL.read();
    Serial.write(c); // This will display "T:...,C:1,M:10,20,30,40" on your screen
    
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        logToSD("RX_ROCKET", inputBuffer);
        
        // Updates continuity based on "C:1" (OK) or "C:0" (Open)
        if (inputBuffer.indexOf("C:1") >= 0) {
          rocketContOK = true;
        } 
        else if (inputBuffer.indexOf("C:0") >= 0) {
          rocketContOK = false;
        }
        
        // Check for safe status
        if (inputBuffer.indexOf("SAFE") >= 0) {
          Serial.println("\n[RECOVERY] ROCKET REPORTS SAFE. TEST COMPLETE.");
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Forward manual commands from keyboard
  if (Serial.available()) {
    char c = Serial.read();
    RYLR_SERIAL.write(c);
  }

  checkSwitches();
}