#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <vector>
#include <SoftwareSerial.h>
#include <stdio.h>
#include "gps_data.h"
// BLE ライブラリ（main.cpp で初期化して使用）
#include "TBGLib.h"

//------------------------------
// BLE
//------------------------------
// BLE ピン定義（Leafony サンプルに合わせる）
#define BLE_WAKEUP      PB12                // D7   PB12
#define BLE_RX          PA1                 // [A2] PA1
#define BLE_TX          PA0                 // [A1] PA0

#define BLE_STATE_STANDBY               (0)
#define BLE_STATE_SCANNING              (1)
#define BLE_STATE_ADVERTISING           (2)
#define BLE_STATE_CONNECTING            (3)
#define BLE_STATE_CONNECTED_MASTER      (4)
#define BLE_STATE_CONNECTED_SLAVE       (5)

bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;
//ローカルデバイス名を設定
String strDeviceName = "Leafony_AC02";

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle
// BLE オブジェクト（BLE UART に HardwareSerial を使用）
HardwareSerial Serialble(BLE_RX, BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );


// プロトタイプ宣言
void setupBLE();
void secondCheck();
// BLE 送信ヘルパーのプロトタイプ
void bt_sendData();

// BLE / BGLib コールバックのプロトタイプ宣言
void onBusy();
void onIdle();
void onTimeout();
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg);
void my_evt_le_connection_opend(const struct ble_msg_le_connection_opend_evt_t *msg);
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg);
void my_evt_system_boot(const struct ble_msg_system_boot_evt_t *msg);
void my_evt_system_awake(void);
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg);

// まだ送信していない gpsLog エントリの次のインデックス
size_t sentIndex = 0;

unsigned long lastTime = 0; // 最後に GPS 情報を更新した時間（ミリ秒）
// TinyGPSPlus オブジェクトを作成
TinyGPSPlus gps;

// "gpsSerial" というソフトウェアシリアルポートを作成
SoftwareSerial gpsSerial(A1, A2); // RX, TX

// プロトタイプ宣言（関数定義が後にあるため）
void displayInfo();

// latestGPS / gpsLog はヘッダで extern 宣言されているのでここで実体を定義する
GPSData latestGPS;
std::vector<GPSData> gpsLog;

void setup() {
  // Arduino のハードウェアシリアルを 115200 ボーレートで開始
  Serial.begin(115200);

  delay(5000); // 起動直後の安定化待ち
  // BLE を初期化
  // setupBLE();
  // ble112.ble_cmd_system_get_bt_address();
  // while (ble112.checkActivity(1000));

  Serial.print("BLE Initialized.\n");

  // GPS のデフォルトボーレートでソフトウェアシリアルを開始
  gpsSerial.begin(9600);
}

void loop() {
  secondCheck();
  // このスケッチは新しい NMEA 文が正しくデコードされるたびに情報を処理
  // ソフトウェアシリアル gpsSerial の受信バッファに読み出せる文字（バイト）がある間、読み取り（非ブロッキング）
  // while (gpsSerial.available() > 0) {
  //   if (gps.encode(gpsSerial.read())) {
  //     displayInfo();
  //   }
    
  // }

  

}

void secondCheck() {
  unsigned long currentMillis = millis();
  // 1 秒ごとに GPS 情報を更新
  if (currentMillis - lastTime >= 1000) {
    Serial.println("1second");
    // if (gps.encode(gpsSerial.read())) {
    //   displayInfo(); 
    // }
    lastTime = currentMillis;
    bt_sendData();
    // BLE: 未送信の gpsLog（日付、緯度、経度）を順次送信
  // for (size_t i = sentIndex; i < gpsLog.size(); ++i) {
    // 日付を YYYYMMDDHHMMSS 形式に整形
    char dateBuf[32];
    // snprintf(dateBuf, sizeof(dateBuf), "%04u%02u%02u%02u%02u%02u",
    //          (unsigned)gpsLog[i].year,
    //          (unsigned)gpsLog[i].month,
    //          (unsigned)gpsLog[i].day,
    //          (unsigned)gpsLog[i].hour,
    //          (unsigned)gpsLog[i].minute,
    //          (unsigned)gpsLog[i].second);

    // 日付を送信
    dateBuf[0] = 's'; 
    dateBuf[1] = 'a';
    dateBuf[2] = 'm';
    dateBuf[3] = 'p';
    dateBuf[4] = 'l';
    dateBuf[5] = 'e';
    dateBuf[6] = '\0';
    size_t len = strlen(dateBuf);

    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8 *)dateBuf);
    while (ble112.checkActivity(1000));

    // // 緯度を 8 バイト double（バイナリ）で送信
    // double latd = gpsLog[i].latitude;
    // Serial.print("BLE Lat (double): "); Serial.println(latd, 6);
    // ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, sizeof(latd), (const uint8 *)&latd);
    // while (ble112.checkActivity(1000));

    // // 経度を 8 バイト double（バイナリ）で送信
    // double lngd = gpsLog[i].longitude;
    // Serial.print("BLE Lng (double): "); Serial.println(lngd, 6);
    // ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, sizeof(lngd), (const uint8 *)&lngd);
    // while (ble112.checkActivity(1000));

    // 送信成功後に sentIndex を進める
    // sentIndex = i + 1;
    // フラッディングを避けるための短い遅延
    // delay(50);
  // }
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


    Serial.print("Lat: "); Serial.println(latestGPS.latitude, 6);
    Serial.print("Lng: "); Serial.println(latestGPS.longitude, 7);  
  // 可変長ログに追加
    gpsLog.push_back(latestGPS);
  }
  // } else {
  //   latestGPS.latitude = 0.0;
  //   latestGPS.longitude = 0.0;
  //   latestGPS.year = 0;
  //   latestGPS.month = 0;
  //   latestGPS.day = 0;
  //   latestGPS.hour = 0;
  //   latestGPS.minute = 0;
  //   latestGPS.second = 0;
  // }
  
  delay(1000);
}

  // -------------------------
  // BLE 初期化（サンプルを簡略化）
  // -------------------------
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
  // // BLE UART を開始
  // Serialble.begin(9600);

  // // 基本ハンドラ（無操作）
  // ble112.onBusy = NULL;
  // ble112.onIdle = NULL;
  // ble112.onTimeout = NULL;

  // // モジュール起動のための短い遅延
  // uint16_t tm = 0;
  // while (tm < 150) {
  //   if (ble112.checkActivity(100) == 0) break;
  //   tm++;
  //   delay(10);
  // }

  // // デバイス名で広告を設定
  // const char *devName = "Leafony_AC02";
  // uint8_t ad_data[31];
  // memset(ad_data, 0, sizeof(ad_data));
  // // 最小構成: ローカル名フィールドを設定
  // size_t nameLen = strlen(devName);
  // ad_data[0] = (uint8_t)(nameLen + 1);
  // ad_data[1] = BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE;
  // for (size_t i = 0; i < nameLen; ++i) ad_data[2 + i] = devName[i];

  // ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, (uint8_t)(nameLen + 2), ad_data);
  // while (ble112.checkActivity(1000));

  // // アドバタイズパラメータを設定して開始
  // ble112.ble_cmd_le_gap_set_adv_parameters(64, 1600, 7);
  // while (ble112.checkActivity(1000));
  // ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);
  // while (ble112.checkActivity(1000));
}
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
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
//=====================================================================
//-----------------------------------------------
// called when the module begins sending a command
void onBusy() {
    // turn LED on when we're busy
    //digitalWrite( D13_LED, HIGH );
}

//-----------------------------------------------
// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    //digitalWrite( D13_LED, LOW );
}

//-----------------------------------------------
// called when the parser does not read the expected response in the specified time limit
void onTimeout() {
    // set state to ADVERTISING
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
// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
}

//-----------------------------------------------
// called immediately after finishing UART TX
void onTXCommandComplete() {
    // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
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

void bt_sendData(){
  char sendData[40];
  sendData[0] = 's';
  sendData[1] = 'a';
  sendData[2] = 'm';
  sendData[3] = 'p';
  sendData[4] = 'l';
  sendData[5] = 'e';
  sendData[6] = '\0';
  uint8_t sendLen;
  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){                                 // BLE transmission
    // Format for WebBluetooth application
    sendLen = sprintf(sendData, "%04u%02u%02u%02u%02u%02u",
                      sendData[0],
                      sendData[1],
                      sendData[2],
                      sendData[3],
                      sendData[4],
                      sendData[5],
                      sendData[6]
                      );
    Serial.println(sendData);
    // Send to BLE device
    ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
    while (ble112.checkActivity(1000));
  }
}