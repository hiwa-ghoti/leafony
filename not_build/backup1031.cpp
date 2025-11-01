#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial
// int RXPin = 2;
// int TXPin = 3;

// Create a TinyGPSPlus object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(A1, A2); // RX, TX

// プロトタイプ宣言（関数定義が後にあるため）
void displayInfo();

//gpsデータ格納用構造体
struct GPSData {
  double latitude;
  double longitude;
  String year;
  String month;
  String day;
  String hour;
  String minute;
  String second;
};

// 最新の GPS データを格納する（外部から参照したい場合のためにグローバル変数を用意）
GPSData latestGPS;

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(9600);
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      displayInfo();
      // 衛星データを受信して表示したら次の受信まで少し待つ
      delay(5000);

      
    }
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    // GPS の値を構造体に格納
    // 注意: struct の year は uint8_t のため、2000 年基準の下位バイト（年 - 2000）で保存します
    latestGPS.latitude = (double)gps.location.lat();
    latestGPS.longitude = (double)gps.location.lng();
    latestGPS.year = (String)(gps.date.year() - 2000);
    latestGPS.month = (String)(gps.date.month());
    latestGPS.day = (String)(gps.date.day());
    latestGPS.hour = (String)(gps.time.hour());
    latestGPS.minute = (String)(gps.time.minute());
    latestGPS.second = (String)(gps.time.second());

  } else {
    latestGPS.latitude = 000.000000;
    latestGPS.longitude = 000.000000;
    latestGPS.year = "0000";
    latestGPS.month = "00";
    latestGPS.day = "00";
    latestGPS.hour = "00";
    latestGPS.minute = "00";
    latestGPS.second = "00";
  }
  
  delay(1000);
}