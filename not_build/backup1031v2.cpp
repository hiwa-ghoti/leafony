#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <vector>
#include <SoftwareSerial.h>
#include <stdio.h>

// Choose two Arduino pins to use for software serial
// int RXPin = 2;
// int TXPin = 3;

// Create a TinyGPSPlus object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(A1, A2); // RX, TX

// プロトタイプ宣言（関数定義が後にあるため）
void displayInfo();

// GPSデータ格納用構造体（String はメモリを圧迫するため数値型で保持）
struct GPSData {
  double latitude;
  double longitude;
  uint16_t year; // 西暦
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

// 最新の GPS データ
GPSData latestGPS;

// 可変長配列（ログ）: std::vector を使用して順次 push_back する
std::vector<GPSData> gpsLog;

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
    }
  }
}

void displayInfo() {
  if (gps.location.isValid()) {
    // GPS の値を構造体に格納（数値で保持）
    latestGPS.latitude = gps.location.lat();
    latestGPS.longitude = gps.location.lng();
    latestGPS.year = (uint16_t)gps.date.year();
    latestGPS.month = (uint8_t)gps.date.month();
    latestGPS.day = (uint8_t)gps.date.day();
    latestGPS.hour = (uint8_t)gps.time.hour();
    latestGPS.minute = (uint8_t)gps.time.minute();
    latestGPS.second = (uint8_t)gps.time.second();

    // 可変長ログに追加
    gpsLog.push_back(latestGPS);
    // 日時をフォーマットして表示
    // 例: 2025/10/31 12:34:56
    char buf[32];
    snprintf(buf, sizeof(buf), "%04u%02u%02u%02u%02u%02u",
             latestGPS.year,
             latestGPS.month,
             latestGPS.day,
             latestGPS.hour,
             latestGPS.minute,
             latestGPS.second);
    Serial.println(buf);
    
  } else {
    latestGPS.latitude = 0.0;
    latestGPS.longitude = 0.0;
    latestGPS.year = 0;
    latestGPS.month = 0;
    latestGPS.day = 0;
    latestGPS.hour = 0;
    latestGPS.minute = 0;
    latestGPS.second = 0;
  }
  
  delay(1000);
}