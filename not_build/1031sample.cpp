#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdio.h>
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

unsigned long lastTime = 0; // 最後に GPS 情報を更新した時間（ミリ秒）

void setup() {
  // Arduino のハードウェアシリアルを 115200 ボーレートで開始
  Serial.begin(115200);

  delay(5000); // 起動直後の安定化待ち
  // BLE を初期化
  // setupBLE();10
  // ble112.ble_cmd_system_get_bt_address();
  // while (ble112.checkActivity(1000));

  Serial.print("BLE Initialized.\n");
}

void loop() {
  secondCheck();
}

void secondCheck() {
  unsigned long currentMillis = millis();
  // 1 秒ごとに GPS 情報を更新
  if (currentMillis - lastTime >= 1000) {
    lastTime = currentMillis;
  bt_sendData();
  // Serial1 はこの環境で未定義の可能性があるため、標準シリアルに出力する
  Serial.println("success");
  }
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

