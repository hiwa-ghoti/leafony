#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <vector>
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
unsigned long previousMillis = 0;  // 前回実行した時刻
const unsigned long interval = 5000; // 5秒（5000ミリ秒）

// GPSデータ格納用構造体（String はメモリを圧迫するため数値型で保持）
struct GPSData {
  double latitude;  //
  double longitude; //度
  double setDate;
};
  char dateBuf[32]; // 日付フォーマット用バッファ

struct DateDate
{
  /* data */
  uint16_t year; // 西暦
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};





// 最新の GPS データ
DateDate setDate;
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
  //ifに変更＆Bluetoothが接続されたときに動作するように変更（フラグ作成予定）
    while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read())){
        displayInfo();
        }
      
    }
}

// void secondCheck() {
//   unsigned long currentMillis = millis();
//   if (currentMillis - lastTime >= 5000) {
//     lastTime = currentMillis;
//     getTime = true;
//   }
// }

void displayInfo() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // 時刻を更新
  // GPS の値を構造体に格納（数値で保持）
    latestGPS.latitude = gps.location.lat();
    latestGPS.longitude = gps.location.lng();
    setDate.year = (uint16_t)gps.date.year();
    setDate.month = (uint8_t)gps.date.month();
    setDate.day = (uint8_t)gps.date.day();
    setDate.hour = (uint8_t)gps.time.hour();
    setDate.minute = (uint8_t)gps.time.minute();
    setDate.second = (uint8_t)gps.time.second();

    // 日時をフォーマットして Serial に表示
    
    snprintf(dateBuf, sizeof(dateBuf), "%04u%02u%02u%02u%02u%02u",
    setDate.year,
    setDate.month,
    setDate.day,
    setDate.hour,
    setDate.minute,
    setDate.second);

    latestGPS.setDate = atof(dateBuf);

    // 可変長ログに追加
    // gpsLog.push_back(latestGPS);
    


    // 位置と日時を表示（位置は小数点6桁で表示）
    Serial.println(latestGPS.setDate);
    Serial.println(latestGPS.latitude);
    Serial.println(latestGPS.longitude, 6);
  }
}