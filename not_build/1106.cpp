#include <Arduino.h> 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <vector>
#include <stdio.h>
#include <string.h>

// Choose two Arduino pins to use for software serial
// int RXPin = 2;
// int TXPin = 3;

// Create a TinyGPSPlus object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(A1, A2); // RX, TX

// プロタイプ宣言（関数定義が後にあるため）
void displayInfo();
void trim(char * data); // 文字列トリム関数のプロトタイプ
unsigned long previousMillis = 0;  // 前回実行した時刻
const unsigned long interval = 5000; // 5秒（5000ミリ秒）

// GPSデータ格納用構造体（String はメモリを圧迫するため数値型で保持）
struct GPSData {
  char latitude[20];  //緯度
  char longitude[20]; //経度
  char setDate[20]; // 日付文字列 (null 終端する) — 19文字 + 終端
};

struct latlngData {
  double latitude;  //緯度
  double longitude; //経度
};

struct DateData
{
  /* data */
  uint16_t year; // 西暦
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

int latLen = 9; // 緯度文字列長
int lngLen = 10; // 経度文字列長
char latBuf[20]; // 緯度フォーマット用バッファ
char lngBuf[20]; // 経度フォーマット用バッファ
char dateBuf[20]; // 日付フォーマット用バッファ

// 最新の GPS データ
latlngData latlng;
DateData Date;
GPSData latestGPS;

// 可変長配列（ログ）: std::vector を使用して順次 push_back する
std::vector<GPSData> gpsLog;

int gpscounter = 0;
int counter = 0;


void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(9600);

  delay(10000);
}

void loop() {
  
  // This sketch displays information every time a new sentence is correctly encoded.
  //ifに変更＆Bluetoothが接続されたときに動作するように変更（フラグ作成予定）
  // こいつはこのままで大丈夫なはず
  if (gpscounter != 10)
  {
    while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read())){
        displayInfo();
        }
    }
  }
  
    

    if (gpscounter == 10 && counter == 0){
      for (size_t i = 0; i < gpsLog.size(); i++)
      {
        Serial.print("GPS Log ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(gpsLog[i].latitude);
        Serial.print(", ");
        Serial.print(gpsLog[i].longitude);
        Serial.print(", ");
        Serial.println(gpsLog[i].setDate);
      }
      counter++;
    }
    
}

// void secondCh > 0) 
//   unsigned long currentMillis = mi 
// Serial.println("-----"); Serial.println("GPSデータ受信完了！");();
//   if (currentMillis - lastT
//     lastTime = currentMillis;
//     getTime = true;
//   }
// }

void displayInfo() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // 時刻を更新
  // GPS の値を構造体に格納（数値で保持）
    latlng.latitude = gps.location.lat();
    latlng.longitude = gps.location.lng();
    Date.year = gps.date.year();
    Date.month = gps.date.month();
    Date.day = gps.date.day();
    Date.hour = gps.time.hour();
    Date.minute = gps.time.minute();
    Date.second = gps.time.second();


    // double 型の緯度経度を文字列に変換
    dtostrf(latlng.latitude, latLen, 6, latBuf);
    dtostrf(latlng.longitude, lngLen, 6, lngBuf);

    // 緯度経度のバッファ内容をデバッグ出力
    // Serial.println(latBuf);
    // Serial.println(lngBuf);

    // 日時をフォーマットしてバッファに格納（"YYYYMMDDHHMMSS"）
    int n = snprintf(dateBuf, sizeof(dateBuf), "%04u%02u%02u%02u%02u%02u",
      (unsigned)Date.year,
      (unsigned)Date.month,
      (unsigned)Date.day,
      (unsigned)Date.hour,
      (unsigned)Date.minute,
      (unsigned)Date.second);

      Serial.println(dateBuf);

    // latestGPSにchar型に変換した各データを格納
    strncpy(latestGPS.latitude, latBuf, sizeof(latestGPS.latitude));
    strncpy(latestGPS.longitude, lngBuf, sizeof(latestGPS.longitude));
    strncpy(latestGPS.setDate, dateBuf, sizeof(latestGPS.setDate));
    
    // Serial.println(latestGPS.setDate);    

    // トリム（空白削除）
    //各データを整形
    trim(latestGPS.latitude);
    trim(latestGPS.longitude);
    trim(latestGPS.setDate);
    

    // 可変長ログに追加
    gpsLog.push_back(latestGPS);
    
    // 位置と日時を表示（位置は小数点6桁で表示）
    // Serial.println(latestGPS.setDate);
    // Serial.println(gpsLog.back().setDate);
    // Serial.println(gpsLog.back().latitude);
    // Serial.println(gpsLog.back().longitude);
    Serial.println(gpsLog.size()); // ログの件数表示
    gpscounter++;
  }
}

void trim(char * data){
  int i = 0, j = 0;

  while (*(data + i) != '\0'){
    if (*(data + i) != ' '){
      *(data + j) = *(data + i);
      j++;
    }
    i++;
  }
  *(data + j) = '\0';
}