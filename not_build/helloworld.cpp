#include <Adafruit_LIS3DH.h>


// シリアルモニターにHello World!を1秒ごとに表示するプログラム
void setup() {
  Serial.begin(115200);
  delay(100);
}

void loop(){
  Serial.println("Hello World!");
  delay(1000);
}