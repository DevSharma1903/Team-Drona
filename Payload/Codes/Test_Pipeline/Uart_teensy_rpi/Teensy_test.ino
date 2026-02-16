void setup() {
  Serial.begin(115200);     // USB debug
  Serial1.begin(115200);    // UART on pins 0 (RX1), 1 (TX1)
}

void loop() {

  // If something comes from Pi
  if (Serial1.available()) {

    String msg = Serial1.readStringUntil('\n');

    Serial.print("Received: ");
    Serial.println(msg);

    // Reply back to Pi
    Serial1.println("PONG");
  }
}