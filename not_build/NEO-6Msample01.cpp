//Lesson 68-2 NEO-6M GPS Sensor
//GPSセンサッモジュールでNSMフォーマトから欲しい情報のみ取り出す
//https://omoroya.com/
#include <Arduino.h>       
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

const int P_RX = 8; //Arduinoで受信するピンの設定
const int P_TX = 7; //設定するが接続しない

TinyGPSPlus gps;                 //gpsというオブジェクトの作成
SoftwareSerial mySS(A1, A2); //mySSというオブジェクトの作成

void setup() {
  Serial.begin(9600); //シリアル通信のスピード(ボーレート)を設定
  mySS.begin(9600);   //mySSシリアル通信のスピード(ボーレート)を設定
}

void loop() {
  //データが有効であれば読み出し
  while (mySS.available() > 0){
    if (gps.encode(mySS.read())){
      Serial.println("2");
      display_monitor();
    }
    else{
      Serial.println("0");
    }
  }
}

void display_monitor() {

  //GPS情報が正常に更新されていたら表示
  if (gps.location.isUpdated()) {

    //年、月、日、衛星数、緯度、経度、高度、対地速度の表示
    Serial.print(gps.date.year());  // 年 (2000+) (u16)
    Serial.print("-");
    Serial.print(gps.date.month()); // 月 (1-12) (u8)
    Serial.print("-");
    Serial.print(gps.date.day());   // 日 (1-31) (u8)
    Serial.print(" SateNum=");
    Serial.print(gps.satellites.value()); //衛星数
    Serial.print(" LAT=");
    Serial.print(gps.location.lat(), 6);  //緯度
    Serial.print(" LNG=");
    Serial.print(gps.location.lng(), 6);  //経度
    Serial.print(" ALT=");
    Serial.print(gps.altitude.meters());  //高度
    Serial.print(" SPEED=");
    Serial.print(gps.speed.kmph());       //対地速度
    Serial.println(" ");
  }
  else{
    Serial.println("1");
  }
}