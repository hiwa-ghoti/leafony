#include <SoftwareSerial.h>
SoftwareSerial mySerial(A1, A2); // RX, TX
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  mySerial.begin(9600);
}

void loop() {
  while (mySerial.available() > 0) {
    char c = mySerial.read();
    Serial.print(c);
  }
}