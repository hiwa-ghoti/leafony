#include <SoftwareSerial.h>
// Define software serial pins for GPS module

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(A1, A2);

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(9600);
}

void loop() {
  // Displays information when a new sentence is available.
  while (gpsSerial.available() > 0)
    Serial.write(gpsSerial.read());
}