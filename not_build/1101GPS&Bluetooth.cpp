//=====================================================================
//  Leafony プラットフォーム サンプルスケッチ
//     アプリケーション: BLE 4センサー デモ
//     プロセッサ    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.13
//     STM32 Core   : Ver1.9.0
//
//     Leaf 構成
//       (1) AC02 BLE Sugar
//       (2) AI01 4-Sensors
//       (3) AP03 STM32 MCU
//       (4) AZ01 USB
//
//    (c) 2021 LEAFONY SYSTEMS 合同会社
//    ライセンス: MIT ライセンス
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 初版
//=====================================================================
//---------------------------------------------------------------------
// 定義セクション
//---------------------------------------------------------------------
#include <Arduino.h>                        // Arduino 基本
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C
#include <stdio.h>
#include <TinyGPSPlus.h>                     // GPS
#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <Adafruit_HTS221.h>                // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor
#include "TBGLib.h"                         // BLE
#include <ST7032.h>                         // LCD
#include <vector>

// --- 関数プロトタイプ宣言 ---
void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data);
void setupPort();
void setupSensor();
void setupBLE();
void setupTimerInt();
void intTimer(void);
void loopCounter();
void loopSensor();
void bt_sendData();
void loopBleRcv(void);
void trim(char * data);
void onBusy();
void onIdle();
void onTimeout();
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg);
void my_evt_le_connection_opend(const struct ble_msg_le_connection_opend_evt_t *msg);
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg);
void my_evt_system_boot(const struct ble_msg_system_boot_evt_t *msg);
void my_evt_system_awake(void);
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg);


//===============================================
// BLE ローカルデバイス名 (Unique Name)
// 最大 16 文字（ASCII）
//===============================================
//                     |1234567890123456|
// ローカルデバイス名を設定
String strDeviceName = "Leafony_AC02";

//===============================================
// シリアルモニタへ出力する設定
//      #define SERIAL_MONITOR でシリアル出力を有効化
//    //コメントアウトするとシリアル出力が無効になります
//===============================================
// 条件付きコンパイルでシリアル出力を切り替えます (下のマクロを定義/コメントアウトしてください)
//#define ← マクロを定義？
#define SERIAL_MONITOR

//===============================================
// デバッグ出力（シリアルモニタ）
//      #define DEBUG でデバッグログを有効化
//    //コメントアウトするとデバッグログは無効になります
//===============================================
//#define DEBUG

//-----------------------------------------------
// 送信間隔の設定
//  SEND_INTERVAL : センサー値（またはダミー日時）を送信する間隔（秒単位）
//-----------------------------------------------
// ここでは 1 秒ごとに送信する設定
#define SEND_INTERVAL   (1)                 // 1s

//-----------------------------------------------
// IO ピン一覧
//-----------------------------------------------
//  D0  PA3  (UART2_RXD)
//  D1  PA2  (UART2_TXD)
//  D2  PC7  (INT0)
//  D3  PB3  (INT1)
//  D4  PB5
//  D5  PB4
//  D6  PA8
//  D7  PB12
//  D8  PA9  (UART1_TX)
//  D9  PA10 (UART1_RX)
//  D10 PB6  (SS)
//  D11 PA7  (MOSI)
//  D12 PA6  (MISO)
//  D13 PA5  (SCK)
//  D14 PB9  (SDA)
//  D15 PB8  (SCL)
//  A0  PA4
//  A1  PA0  (UART4_TX)
//  A2  PA1  (UART4_RX)
//  A3  PB0
//  A4  PC1
//  A5  PC0

//-----------------------------------------------
// IO ピン名の定義
// 接続する Leaf に合わせて定義を調整してください。
//-----------------------------------------------
#define BLE_WAKEUP      PB12                // D7   PB12
#define BLE_RX          PA1                 // [A2] PA1
#define BLE_TX          PA0                 // [A1] PA0

//------------------------------
// ループ間隔
// タイマー割り込み間隔（ミリ秒単位）
//------------------------------
#define LOOP_INTERVAL 125000                // 125000us = 125ms interval

//------------------------------
// BLE
//------------------------------
#define BLE_STATE_STANDBY               (0)
#define BLE_STATE_SCANNING              (1)
#define BLE_STATE_ADVERTISING           (2)
#define BLE_STATE_CONNECTING            (3)
#define BLE_STATE_CONNECTED_MASTER      (4)
#define BLE_STATE_CONNECTED_SLAVE       (5)

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------

//------------------------------
// GPS
//------------------------------
SoftwareSerial gpsSerial(A1, A2); // RX, TX
// GPSデータ格納用構造体
struct GPSData {
  double latitude;
  double longitude;
  uint16_t year; // 西暦
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  double setDate;
};
TinyGPSPlus gps;
GPSData latestGPS;
std::vector<GPSData> gpsLog;  // 可変長配列（ログ）: std::vector を使用して順次 push_back する
unsigned long previousMillis = 0;  // 前回実行した時刻
const unsigned long interval = 5000; // 5秒（5000ミリ秒）

struct sampleData {
  double latitude;
  double longitude;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

//------------------------------
// BLE
//------------------------------
//STM32版ArduinoのHardwareSerialコンストラクタ
//BLEモジュールとUART接続のためのシリアルポートオブジェクトを作成
//その時BLE_RX(PA1), BLE_TX(PA0)ピンを使用
HardwareSerial Serialble(BLE_RX, BLE_TX);
//ble112はBGLib(TBGlib)のインスタンス
//Serialbleを使ってBLEモジュールへコマンド送信/応答受信/イベント処理を行う
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//------------------------------
// Loop counter
//------------------------------
uint8_t iLoop5s = 0;
uint8_t iSendCounter = 0;

//------------------------------
// Event
//------------------------------
bool event1s = false;

//------------------------------
// interval Timer2 interrupt
//------------------------------
volatile bool bInterval = false;

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

// ダミー日時カウンタ（センサーの代わりに送信する日時）
uint16_t dummy_year = 2025;
uint8_t dummy_month = 1;
uint8_t dummy_day = 1;
uint8_t dummy_hour = 0;
uint8_t dummy_minute = 0;
uint8_t dummy_second = 0;

//=====================================================================
// setup
//=====================================================================
void setup(){
  Serial.begin(115200);     // UART 115200bps

  Wire.begin();             // I2C初期化（各センサー処理に必要）おそらく加速度センサー処理に使用
  setupPort();              // BLEのスリープ状態解除
  setupBLE();               // BLE初期化

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));

  // STM32のハードウェアタイマーを使って125msで割り込みを発生させる
  // ソフト側で一秒周期を安定して行うために必要
  setupTimerInt();                           // タイマー割り込み開始

}

//-----------------------------------------------
// IO ピンの入出力設定
// 接続する Leafony に合わせて設定を行ってください
//-----------------------------------------------
void setupPort(){
  //BLE_WAKEUP(PB12)を出力ピンとして設定
  pinMode(BLE_WAKEUP, OUTPUT);           // [D7] : BLE Wakeup/Sleep
  //その出力ピン(BLE_WAKEUP)をHIGHに設定してBLEモジュールを起動
  digitalWrite(BLE_WAKEUP, HIGH);        // BLE Wakeup
}

// タイマー割り込みのセットアップ（125ms 間隔）
void setupTimerInt(){
  HardwareTimer *timer2 = new HardwareTimer (TIM2);

  timer2->setOverflow(LOOP_INTERVAL, MICROSEC_FORMAT);       // 125ms
  timer2->attachInterrupt(intTimer);
  timer2->resume();
}

//====================================================================
// Loop
//====================================================================
void loop() {
  while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read())){
          setDate();
        }
  }

  
  loopBleRcv();
}
//---------------------------------------------------------------------
// ループカウンタ
// メインループのループ回数をカウントし、5秒間隔でセンサー取得やBLE送信のフラグを立てます
//---------------------------------------------------------------------
void loopCounter(){
  iLoop5s += 1;

  //--------------------
  // 5s period
  //--------------------
  if (iLoop5s >=  40){                 // 125ms x 40 = 5s
    iLoop5s = 0;

    iSendCounter  += 1;
    if (iSendCounter >= SEND_INTERVAL){
      iSendCounter = 0;
      event1s = true;
    }
  }
}

void setDate(){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // 時刻を更新
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
    // 日時をフォーマットして Serial に表示
    char dateBuf[32];
    snprintf(dateBuf, sizeof(dateBuf), "%04u%02u%02u%02u%02u%02u",
             latestGPS.year,
             latestGPS.month,
             latestGPS.day,
             latestGPS.hour,
             latestGPS.minute,
             latestGPS.second);

    // 位置と日時を表示（位置は小数点6桁で表示）
    Serial.println(dateBuf);
    Serial.println(latestGPS.latitude, 6);
    Serial.println(latestGPS.longitude, 6);
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
}

//---------------------------------------------------------------------
// ダミー日時を BLE で送信
// センサー取得/送信処理を無効化し、ダミーの日時（YYYYMMDDHHMMSS）を送信します。
//---------------------------------------------------------------------
void bt_sendData(){
  for(int i = 0; i < gpsLog.size(); i++) {
    Serial.println("GPS Log Entry:");


  }
  char dateBuf[32];

  int len = snprintf(dateBuf, sizeof(dateBuf), "%04u%02u%02u%02u%02u%02u",
                     (unsigned)dummy_year, (unsigned)dummy_month, (unsigned)dummy_day,
                     (unsigned)dummy_hour, (unsigned)dummy_minute, (unsigned)dummy_second);


  // シリアルとBLEで送信（デバッグ用に常に出力）
  Serial.println(dateBuf);

  // BLE へ送信（通知キャラクタリスティック 0x000C を使用）
  ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, (uint8_t)len, (const uint8 *)dateBuf);
  while (ble112.checkActivity(1000));
}
//====================================================================

//==============================================
// Interrupt
//==============================================

//-----------------------------------------------
// BLE からデータが送られてきた場合に受信して処理を行う
//-----------------------------------------------
//void intTimer(HardwareTimer*){      // STM32 Core 1.7.0
void intTimer(void){                  // STM32 Core 1.9.0
  bInterval = true;
}

//---------------------------------------
// trim
// Removing SP from a string array
//---------------------------------------
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

//=====================================================================
// I2C control function
//=====================================================================
//-----------------------------------------------
// I2C Write 1 byte to the slave device
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
// Read 1 byte from the slave device
//-----------------------------------------------
unsigned char i2c_read_byte(int device_address, int reg_address){
  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}

//=====================================================================
// BLE
//=====================================================================
//-----------------------------------------------
//  Setup BLE
//-----------------------------------------------
void setupBLE(){
    uint8  stLen;
    uint8 adv_data[31];

    // set up internal status handlers (these are technically optional)
    ble112.onBusy = onBusy;
    ble112.onIdle = onIdle;
    ble112.onTimeout = onTimeout;
    // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
    // BLE module must be woken up before sending any UART data

    // set up BGLib response handlers (called almost immediately after sending commands)
    // (these are also technicaly optional)

    // set up BGLib event handlers
    /* [gatt_server] */
    ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;    /* [BGLib] */
    /* [le_connection] */
    ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;                    /* [BGLib] */
    ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;                  /* [BGLib] */
    /* [system] */
    ble112.ble_evt_system_boot = my_evt_system_boot;                                    /* [BGLib] */

    ble112.ble_evt_system_awake = my_evt_system_awake;
    ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

    uint8_t tm=0;
    Serialble.begin(9600);
    while (!Serialble && tm <150){                              // Wait for Serial to start Timeout 1.5s
      tm++;
      delay(10);
    }

    tm=0;
    while (!bSystemBootBle && tm <150){                         // Waiting for BLE to start
      ble112.checkActivity(100);
      tm++;
      delay(10);
    }

    /* setting */
    /* [set Advertising Data] */
    uint8 ad_data[21] = {
        (2),                                                    // field length
        BGLIB_GAP_AD_TYPE_FLAGS,                                // field type (0x01)
        (6),                                                    // data
        (1),                                                    // field length (1は仮の初期値)
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE                    // field type (0x09)
    };

    /*  */
    size_t lenStr2 = strDeviceName.length();

    ad_data[3] = (lenStr2 + 1);                                 // field length
    uint8 u8Index;
    for( u8Index=0; u8Index < lenStr2; u8Index++){
      ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
    }

    /*   */
    stLen = (5 + lenStr2);

    //ble112.ble_cmd_le_gap_bt5_set_adv_data(0,SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);
    ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);

    while (ble112.checkActivity(1000));                         /* Receive check */
    delay(20);

    /* interval_min :   40ms( =   64 x 0.625ms ) */
    //ble112.ble_cmd_le_gap_bt5_set_adv_parameters( 0, 64, 1600, 7, 0 );/* [BGLIB] <handle> <interval_min> <interval_max> <channel_map> <report_scan>*/
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */

    while (ble112.checkActivity(1000));                         /* [BGLIB] Receive check */

    /* start */
//    ble112.ble_cmd_le_gap_bt5_set_mode(0,LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE,0,2);
//    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);     // index = 0
    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);                // index = 0
    while (ble112.checkActivity(1000));                         /* Receive check */
    /*  */
}

//-----------------------------------------
// If data is sent from the BLE, acquire the data
// and perform processing according to the acquired data.
//-----------------------------------------
void loopBleRcv(void){
    // keep polling for new data from BLE
    ble112.checkActivity(0);                                    /* Receive check */

    /*  */
    if (ble_state == BLE_STATE_STANDBY) {
        bBLEconnect = false;                                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_ADVERTISING) {
        bBLEconnect = false;                                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
        /*  */
        bBLEconnect = true;                                     /* [BLE] connection state */
    }
}

//=====================================================================
// 内部 BGLib クラス コールバック関数群
//=====================================================================
//-----------------------------------------------
// モジュールがコマンド送信を開始したときに呼ばれる
void onBusy() {
  // 処理中は（必要に応じて）LED を点灯する処理をここに書けます
  //digitalWrite( D13_LED, HIGH );
}

//-----------------------------------------------
// モジュールが応答や "system_boot" イベントを受け取ったときに呼ばれる
void onIdle() {
  // 処理が終わったら（必要に応じて）LED を消灯する処理をここに書けます
  //digitalWrite( D13_LED, LOW );
}

//-----------------------------------------------
// パーサが指定時間内に期待する応答を受け取れなかったときに呼ばれる
void onTimeout() {
  // 状態を ADVERTISING に設定
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                                        /* [BLE] connection state */
    bBLEsendData = false;
#ifdef DEBUG
     Serial.println(F("on time out"));
#endif
}

//-----------------------------------------------
// コマンドの UART 送信を開始する直前に呼ばれる
void onBeforeTXCommand() {
}

//-----------------------------------------------
// コマンドの UART 送信が完了した直後に呼ばれる
void onTXCommandComplete() {
  // (例) 送信完了後にモジュールをスリープに戻す処理を書く
#ifdef DEBUG
    Serial.println(F("onTXCommandComplete"));
#endif
}
/*  */

//-----------------------------------------------
void my_evt_gatt_server_attribute_value( const struct ble_msg_gatt_server_attribute_value_evt_t *msg ) {
    uint16 attribute = (uint16)msg -> attribute;
    uint16 offset = 0;
    uint8 value_len = msg -> value.len;
    uint8 value_data[20];
    String rcv_data;
    rcv_data = "";
    for (uint8_t i = 0; i < value_len; i++) {
        rcv_data += (char)(msg -> value.data[i]);
    }

#ifdef DEBUG
        Serial.print(F("###\tgatt_server_attribute_value: { "));
        Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
        Serial.print(F(", attribute: ")); Serial.print((uint16_t)msg -> attribute, HEX);
        Serial.print(F(", att_opcode: ")); Serial.print(msg -> att_opcode, HEX);

        Serial.print(", offset: "); Serial.print((uint16_t)msg -> offset, HEX);
        Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
        Serial.print(", value_data: "); Serial.print(rcv_data);

        Serial.println(F(" }"));
#endif

    if( rcv_data.indexOf("SND") == 0 ){
        bBLEsendData = true;
    } else if( rcv_data.indexOf("STP") == 0 ){
        bBLEsendData = false;
    }
}
/*  */

//-----------------------------------------------
void my_evt_le_connection_opend( const ble_msg_le_connection_opend_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_opend: { "));
        Serial.print(F("address: "));
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) Serial.write('0');
            Serial.print(msg -> address.addr[i], HEX);
        }
        Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
        Serial.print(", master: "); Serial.print(msg -> master, HEX);
        Serial.print(", connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
        Serial.print(", advertiser: "); Serial.print(msg -> advertiser, HEX);
        Serial.println(" }");
    #endif
    /*  */
    ble_state = BLE_STATE_CONNECTED_SLAVE;
}
/*  */
//-----------------------------------------------
void my_evt_le_connection_closed( const struct ble_msg_le_connection_closed_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_closed: { "));
        Serial.print(F("reason: ")); Serial.print((uint16_t)msg -> reason, HEX);
        Serial.print(F(", connection: ")); Serial.print(msg -> connection, HEX);
        Serial.println(F(" }"));
    #endif

    // after disconnection, resume advertising as discoverable/connectable (with user-defined advertisement data)
//    ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);              // index = 0
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);   // index = 0
//     ble112.ble_cmd_le_gap_set_mode(LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE );
     ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE); 

    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                                        /* [BLE] connection state */
    bBLEsendData = false;
}
/*  */

//-----------------------------------------------
void my_evt_system_boot( const ble_msg_system_boot_evt_t *msg ){
    #ifdef DEBUG
        Serial.print( "###\tsystem_boot: { " );
        Serial.print( "major: " ); Serial.print(msg -> major, HEX);
        Serial.print( ", minor: " ); Serial.print(msg -> minor, HEX);
        Serial.print( ", patch: " ); Serial.print(msg -> patch, HEX);
        Serial.print( ", build: " ); Serial.print(msg -> build, HEX);
        Serial.print( ", bootloader_version: " ); Serial.print( msg -> bootloader, HEX );           /*  */
        Serial.print( ", hw: " ); Serial.print( msg -> hw, HEX );
        Serial.println( " }" );
    #endif

     bSystemBootBle = true;

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

//-----------------------------------------------
void my_evt_system_awake(void){
  ble112.ble_cmd_system_halt( 0 );
  while (ble112.checkActivity(1000));
}

//-----------------------------------------------
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg ){
#ifdef DEBUG
  Serial.print( "###\tsystem_get_bt_address: { " );
  Serial.print( "address: " );
  for (int i = 0; i < 6 ;i++){
    Serial.print(msg->address.addr[i],HEX);
  }
  Serial.println( " }" );
#endif

#ifdef SERIAL_MONITOR
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] *0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ",addr);
  Serial.println(cAddr);
#endif
}
// --- 未定義関数・変数のダミー定義 ---
void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {}
 