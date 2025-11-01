#include <Adafruit_LIS3DH.h>

//LIS3DHという加速度センサーのI2Cアドレスを定義
#define LIS3DH_ADDRESS 0x19


Adafruit_LIS3DH accel = Adafruit_LIS3DH();

void setup() {
  Serial.begin(115200);

  //アドレスを指定してセンサーとの通信を開始
  accel.begin(LIS3DH_ADDRESS);

  // 割り込み機能を無効化  
  //LIS3DHには割り込み機能（タップやダブルタップ）があるが、今回は使わない
  accel.setClick(0, 0);                      
  // センサーの計測範囲を±2Gに設定   
  //2Gは日常の動きから激しい動きまで計測可能
  //Gが小さくなるほど計測範囲は狭まるが精度が上がる
  accel.setRange(LIS3DH_RANGE_2_G);          
  // データ取得周期を10Hzに設定  
  //1秒間に10回データを取得する設定
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);  

  delay(100);
}

void loop() {
  // センサーから最新のデータを読み取る
  accel.read();

  Serial.print("X [g] = " + String(accel.x_g));
  Serial.print(", ");
  Serial.print("Y [g] = " + String(accel.y_g));
  Serial.print(", ");
  Serial.print("Z [g] = " + String(accel.z_g));
  Serial.println("");

  delay(100);
}