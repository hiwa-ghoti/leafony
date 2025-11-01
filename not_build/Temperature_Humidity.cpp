#include <Adafruit_LIS3DH.h>

#include <Wire.h>
#include <HTS221.h>

//---------------------------
// センサーの値を、実際の正しい値に近づけるための補正式を作るためのデータ定義
//---------------------------
// 温度補正データ0
float TL0 = 25.0;     // 4-Sensorsの温度測定値
float TM0 = 25.0;     // 温度計や他の測定値
// 温度補正データ1
float TL1 = 40.0;     // 4-Sensorsの温度測定値
float TM1 = 40.0;     // 温度計や他の測定値

// 湿度補正データ0
float HL0 = 60.0;     // 4-Sensorsの湿度測定値
float HM0 = 60.0;     // 湿度計や他の測定値
// 湿度補正データ1
float HL1 = 80.0;     // 4-Sensorsの湿度測定値
float HM1 = 80.0;     // 湿度計や他の測定値

void setup() {
  // シリアル通信を115200bpsで初期化
  Serial.begin(115200);
  Wire.begin();             // I2C通信を初期化　// I2Cとは、STM32と4-Sensorsの通信方式
  smeHumidity.begin();      // HTS221センサーを初期化
  delay(10);
}

void loop() {
  // read temperature and humidity:
  float dataTemp = (float)smeHumidity.readTemperature();  // HTS221から温度データを取得
  float dataHumid = (float)smeHumidity.readHumidity();    // HTS221から湿度データを取得

  // calibration:
  dataTemp = TM0 + (TM1 - TM0) * (dataTemp - TL0) / (TL1 - TL0);      // 温度補正
  dataHumid = HM0 + (HM1 - HM0) * (dataHumid - HL0) / (HL1 - HL0);    // 湿度補正

  Serial.println(String(dataTemp) + "[℃], " + String(dataHumid) + "[%]");
  delay(1000);
}
//これからやること補正の修正