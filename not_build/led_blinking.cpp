#include <Adafruit_LIS3DH.h>

// LEDを1秒ごとに点滅させるプログラム
void setup() {

 pinMode(PA5_ALT0,OUTPUT);
}


void loop() {

 digitalWrite(PA5_ALT0,LOW);
 delay(1000);
 digitalWrite(PA5_ALT0,HIGH);
 delay(1000);
}