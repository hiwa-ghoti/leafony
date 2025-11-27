//---------------------------------------------------------------------
// 定義
//---------------------------------------------------------------------
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C

#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <Adafruit_HTS221.h>                // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor
#include "TBGLib.h"                         // BLE
#include <ST7032.h>                         // LCD
#include <TinyGPSPlus.h>
#include <vector>

// --- 関数プロトタイプ宣言 ---
// BLE functions
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
//music functions
void initializeNote () ;
void initializeAcceleSensor () ;
void getHeight ( double height ) ;
void getVibration ( double level ) ;
int playMusic ( int id, double ts ) ;
void timer100 () ;
void timer500 () ;
void timer1000 () ;
void timer5000 () ;
void timer10000 () ;
void timer30000 () ;




//===============================================
//BLE一意の名前（ローカルデバイス名）
//最大16文字（ASCIIコード）
//===============================================
//                     |1234567890123456|
//ローカルデバイス名を設定
String strDeviceName = "Leafony_AC02";

//===============================================
// シリアルモニターに出力
//      #define SERIAL_MONITOR = 出力あり
//    //#define SERIAL_MONITOR = 出力なし（コメントアウト）
//===============================================
// 事前定義存在定義(#ifdef)で用いる
//#define ← マクロを定義？
#define SERIAL_MONITOR

//===============================================
// デバッグ出力をシリアルモニターに出力
//      #define DEBUG = 出力あり
//    //#define DEBUG = 出力なし（コメントアウト）
//===============================================
//#define DEBUG

//-----------------------------------------------
// 送信間隔の設定
//  SEND_INTERVAL  :送信間隔（センサーデータの送信間隔を1秒単位で設定します。）
//-----------------------------------------------
//送信間隔を1秒単位で設定するマクロ
//1秒に1回送信
#define SEND_INTERVAL   (1)                 // 1s

//-----------------------------------------------
// IOピン名定義
// 接続するリーフに合わせて定義してください。
//-----------------------------------------------
#define BLE_WAKEUP      PB12                // D7   PB12
#define BLE_RX          PA1                 // [A2] PA1
#define BLE_TX          PA0                 // [A1] PA0

//-----------------------------------------------
// 定数定義
//-----------------------------------------------
//------------------------------
// I2Cアドレス
//------------------------------
#define LIS2DH_ADDRESS          0x19        // 加速度センサー (SD0/SA0ピン = VCC)
#define OPT3001_ADDRESS         0x45        // 環境光センサー (ADDRピン = VCC)
#define LCD_I2C_EXPANDER_ADDR   0x1A        // LCD I2Cエキスパンダー
#define BATT_ADC_ADDR           0x50        // バッテリーADC

//------------------------------
// ループ間隔
// タイマー割り込み間隔（ミリ秒）
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

// ------------------------------
// GPS
// ------------------------------
TinyGPSPlus gps;
SoftwareSerial gpsSerial(8, 9); // RX, TX
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


// ------------------------------
// music
// ------------------------------
#define LIS3DH_ADDRESS 0x19
Adafruit_LIS3DH accel = Adafruit_LIS3DH ();

double GPSHeight = 0.0 ;  
int usePinCount = 8 ;
int usePin[] = { D0, D4, D6, D7, D8, D10, D11, D12 } ; 
int DO = 0 ;
int RE = 1 ;
int MI = 2 ;
int FA = 3 ;
int SO = 4 ;
int RA = 5 ;
int SI = 6 ;
int DO2 = 7 ;
int NO = 8 ;
int END = 9 ;

int musicMode = 0 ;
int music[][256] = {
    { DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 12, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, FA, 18, NO, 30, END
    },
    { RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, 
      MI, 6, FA, 6, MI, 6, RE, 6, FA, 18, NO, 30, END
    },
    { FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 12, 
      MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6, 
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12, NO, 30, END
    },  
    { MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, RE, 6, MI, 6, FA, 6, MI, 12,
      FA, 6, MI, 6, RE, 6, DO, 6, RE, 6, MI, 6, FA, 6, MI, 6, 
      RE, 6, DO, 6, RE, 6, MI, 6, FA, 18, NO, 30, END
    },
    { DO, 6, NO, 6, RE, 6, MI, 6, FA, 6, MI, 6, RE, 6,
      DO, 6, NO, 6, MI, 6, NO, 6, FA, 6, NO, 6, MI, 6, NO, 6, 
      DO, 6, RE, 6, NO, 6, MI, 6, FA, 6, NO, 6, RE, 6, NO, 6, 
      DO, 6, MI, 6, FA, 12, NO, 30, END
    },
    { MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, MI, 6, RE, 6, DO, 6, NO, 6, RE, 6, NO, 6, 
      MI, 6, NO, 6, FA, 6, RE, 6, DO, 6, NO, 6, RE, 6, MI, 6,
      FA, 6, NO, 6, RE, 6, NO, 6, DO, 6, MI, 6, FA, 12, NO, 30, END
    },
    { NO, 100, END }
} ;

int orderCount = 50 ;
int order[] = {
     0, 0, 0, 0, 6, 0, 0, 0, 0, 6, 
     1, 1, 1, 1, 6, 1, 1, 1, 1, 6, 
     2, 2, 2, 2, 6, 2, 2, 2, 2, 6,
     3, 3, 3, 3, 6, 3, 3, 3, 3, 6,
     4, 4, 4, 4, 6, 5, 5, 5, 5, 6 } ;
double speed[] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.75, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.75, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.75, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.75, 1.0,
    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
} ;

int accMusic = 0 ;
int upMusic = 0 ;
int downMusic = 0 ;
int selectMusic = 0 ;

//------------------------------
// LCD
//------------------------------
ST7032 lcd;

//------------------------------
// Sensor
//------------------------------
// Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

//------------------------------
// BLE
//------------------------------
HardwareSerial Serialble(BLE_RX, BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//---------------------------------------------------------------------
// プログラムで使用する変数を定義する
//---------------------------------------------------------------------
//------------------------------
// LCD
//------------------------------
bool dispLCD = 0;                           // Set to 1 to display on LCD.
int8_t lcdSendCount = 0;

//------------------------------
// Loop counter
//------------------------------
uint8_t iLoop1s = 0;
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
// LIS2DH : Accelerometer
//------------------------------
float dataX_g, dataY_g, dataZ_g;
float dataTilt = 0;
uint8_t dataPips;

//------------------------------
// HTS221 : Humidity and Temperature sensor
//------------------------------
Adafruit_HTS221 hts;

float dataTemp = 0;
float dataHumid = 0;

//--------------------
// Data for two-point correction
//--------------------
// Temperature correction data 0
float TL0 = 25.0;     // 4-Sensors Temperature measurement value
float TM0 = 25.0;     // Thermometer and other measurements value
// Temperature correction data 1
float TL1 = 40.0;     // 4-Sensors Temperature measurement value
float TM1 = 40.0;     // Thermometer and other measurements value

// Humidity correction data 0
float HL0 = 60.0;     // 4-Sensors Humidity measurement value
float HM0 = 60.0;     // Hygrometer and other measurements value
// Humidity correction data 1
float HL1 = 80.0;     // 4-Sensors Humidity measurement value
float HM1 = 80.0;     // Hygrometer and other measurements value

//------------------------------
// OPT3001 : Ambient Light Sensor
//------------------------------
float dataLight;

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

//------------------------------
// Battery
//------------------------------
float dataBatt = 0;

//=====================================================================
// setup
//=====================================================================
void setup(){

  Serial.begin(9600);     // UART 9600bps
  gpsSerial.begin(9600);
  Wire.begin();             // I2C 100kHz
//SERIAL_MONITORがあるかどうかで分岐


  delay(10000);
  //ダミーデータ作成
  //   /* 琵琶湖 */
  // strcpy(latestGPS.setDate, "20251763000000");
  // strcpy(latestGPS.latitude, "35.015528");
  // strcpy(latestGPS.longitude, "135.862333");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000010");
  // strcpy(latestGPS.latitude, "35.015528");
  // strcpy(latestGPS.longitude, "135.862333");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000020");
  // strcpy(latestGPS.latitude, "35.006414");
  // strcpy(latestGPS.longitude, "135.882016");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000030");
  // strcpy(latestGPS.latitude, "34.997300");
  // strcpy(latestGPS.longitude, "135.901700");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000040");
  // strcpy(latestGPS.latitude, "34.985177");
  // strcpy(latestGPS.longitude, "135.903947");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000050");
  // strcpy(latestGPS.latitude, "34.985177");
  // strcpy(latestGPS.longitude, "135.903947");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000060");
  // strcpy(latestGPS.latitude, "34.973054");
  // strcpy(latestGPS.longitude, "135.906195");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000070");
  // strcpy(latestGPS.latitude, "34.973054");
  // strcpy(latestGPS.longitude, "135.906195");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000080");
  // strcpy(latestGPS.latitude, "35.023332");
  // strcpy(latestGPS.longitude, "135.920877");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000090");
  // strcpy(latestGPS.latitude, "35.073610");
  // strcpy(latestGPS.longitude, "135.935560");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000100");
  // strcpy(latestGPS.latitude, "35.073610");
  // strcpy(latestGPS.longitude, "135.935560");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000110");
  // strcpy(latestGPS.latitude, "35.097738");
  // strcpy(latestGPS.longitude, "135.935275");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000120");
  // strcpy(latestGPS.latitude, "35.121866");
  // strcpy(latestGPS.longitude, "135.934991");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000130");
  // strcpy(latestGPS.latitude, "35.121866");
  // strcpy(latestGPS.longitude, "135.934991");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000140");
  // strcpy(latestGPS.latitude, "35.068697");
  // strcpy(latestGPS.longitude, "135.898662");
  // gpsLog.push_back(latestGPS);
  // strcpy(latestGPS.setDate, "20251763000150");
  // strcpy(latestGPS.latitude, "35.015528");
  // strcpy(latestGPS.longitude, "135.862333");
  // gpsLog.push_back(latestGPS);

  initializeNote () ;
  
  setupPort();
  Serial.println("SetupPort");
  setupSensor();
  Serial.println("SetupSensor");
  setupBLE();
  Serial.println("SetupBLE");

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));

  setupTimerInt();                            // Timer inverval start
}

//-----------------------------------------------
// IO ピンの入出力設定
//-----------------------------------------------
void setupPort(){
  //BLE_WAKEUP(PB12)を出力ピンとして設定
  pinMode(BLE_WAKEUP, OUTPUT);           // [D7] : BLE Wakeup/Sleep
  //その出力ピン(BLE_WAKEUP)をHIGHに設定してBLEモジュールを起動
  digitalWrite(BLE_WAKEUP, HIGH);        // BLE Wakeup
}

//-----------------------------------------------
// Timer interrupt (interval=125ms, int=overflow)
// Timer interrupt setting for main loop
//-----------------------------------------------
void setupTimerInt(){
  HardwareTimer *timer2 = new HardwareTimer (TIM2);

  timer2->setOverflow(LOOP_INTERVAL, MICROSEC_FORMAT);       // 125ms
  timer2->attachInterrupt(intTimer);
  timer2->resume();
}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //----------------------------
  // LIS2DH (accelerometer)
  //----------------------------
  accel.begin(LIS2DH_ADDRESS);
  accel.setClick(0, 0);                      // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);          // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);  // Data rate = 10Hz
  delay ( 100 ) ;

  //----------------------------
  // HTS221 (temperature / humidity)
  //----------------------------
  while (!hts.begin_I2C()) {
#ifdef SERIAL_MONITOR
    Serial.println("Failed to find HTS221");
#endif
    delay(10);
  }

  //----------------------------
  // OPT3001 (light)
  //----------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);

  newConfig.RangeNumber = B1100;                    // automatic full scale
  newConfig.ConvertionTime = B1;                    // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;        // continous conversion
  newConfig.Latch = B0;                             // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);     // retry
  }
}

// music functions
void initializeNote ()
{
  Serial.println("Initialize Note");
    int     i ;

    for ( i = 0 ; i < usePinCount ; i ++ )
    {
        pinMode ( usePin[i], OUTPUT ) ; //使用するピンを出力モードに設定
        digitalWrite ( usePin[i], HIGH ) ;  //ピンをHIGHに設定（おはようモード）
    } 
    delay ( 100 ) ;       
}

//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop() {
  // Serial.println("Goodmorning");
  timer100 () ;
  timer1000 () ;
  timer30000 () ;
  
  // GPSデータの取得と表示
  if(bBLEsendData == false){
    // Serial.println("GPS Read");
    while (gpsSerial.available() > 0){
        if (gps.encode(gpsSerial.read())){
        displayInfo();
        }
    }
  }

  // bIntervalがtrueかつbBLEsendDataがtrueのときにデータ送信を実行
  if (bInterval == true && bBLEsendData == true){
     bInterval = false;
    //これをなくしたら動かくなるため必要
    //おそらくevent1sがtrueになるまで待っている状態になるため  
    loopCounter();                    // loop counter
    if(event1s == true){
      event1s = false;
      Serial.println("entry sensor");
      loopSensor();                   // sensor read
      //ここでデータを送信する
      bt_sendData();                  // Data send
      Serial.println("send data");
    }
  }
  loopBleRcv();
}

void getHeight ( double height )
{
    static double   oldHeight = -1.0 ;

    if ( oldHeight < 0.0 )
    {
        oldHeight = GPSHeight ;
    }
    if ( GPSHeight - oldHeight > height )
    {
        upMusic = 1 ;
        selectMusic = rand () % 2 ;
        Serial.print ( "Up Music Start" ) ; 
    }
    else if ( GPSHeight - oldHeight < -height )
    {
        downMusic = 1 ;
        selectMusic = rand () % 2 ; 
        Serial.print ( "Up Music Start" ) ; 
    }
    else
    {
        upMusic = 0 ;
        downMusic = 0 ;
    }
    oldHeight = GPSHeight ;
}

void getVibration ( double level )
{
    double  acc ;
    static int count = 0 ;
    static double oldAcc = -1.0 ;

    accel.read () ;
    acc = sqrt ( accel.x_g * accel.x_g + accel.y_g * accel.y_g + accel.z_g * accel.z_g ) ; 
    if ( oldAcc < 0.0 )
    {
        oldAcc = acc ;
    }
    if ( fabs ( acc - oldAcc ) > level )
    {
        count ++ ;
    }
    else
    {
        count = 0 ;
        if ( accMusic == 1 )
        {
            accMusic = 0 ;
            Serial.print ( "Accel Music Stop\n" ) ;             
        }
    }
    if ( count > 5 && accMusic == 0 )
    {
        accMusic = 1 ; 
        Serial.print ( "Accel Music Start\n" ) ;
        selectMusic = rand () % 2 ;       
    }
    oldAcc = acc ;    
}

int playMusic ( int id, double ts )
{
    static int t = 0 ;
    static int tcount = 0 ;

    if ( music[id][t*2] == END )
    {
        t = 0 ;
        return 1 ;
    }

    if ( tcount == 0 )
    {
        if ( music[id][t*2] != NO )
        {
            digitalWrite ( usePin[music[id][t*2]], LOW ) ;
            delay ( ( int )( music[id][t*2+1] * 100.0 * ( 0.005 * ts ) ) ) ;
            digitalWrite ( usePin[music[id][t*2]], HIGH ) ;
        }
    }   
    tcount ++ ;

    if ( tcount >= ( int ) ( music[id][t*2+1] * ts ) )
    {
        tcount = 0 ;
        t ++ ;
    }
    return 0 ;
}

/*
 *  timer func 0.1s
 */
void timer100 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;
    static int id = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 100 )
    {
        if ( playMusic ( order[id], speed[id] ) == 1 )
        {
            id ++ ;
            id = id % orderCount ;
        }
        oldTime = currentTime ;
    }
}

/*
 *  timer func 0.5s
 */
void timer500 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 500 )
    {

        oldTime = currentTime ;
    }
}

/*
 *  timer func 1s
 */
void timer1000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 1000 )
    {
        getVibration ( 0.05 ) ;
        oldTime = currentTime ;
    }
}

/*
 *  timer func 5s
 */
void timer5000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 5000 )
    {
        oldTime = currentTime ;
    }
}

/*
 *  timer func 10s
 */
void timer10000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 10000 )
    {
        getHeight ( 0.5 ) ;
        Serial.print ( "Updown Music Start" ) ;
        oldTime = currentTime ;
    }
}

/*
 *  timer func 30s
 */
void timer30000 ()
{
    int currentTime = millis () ;
    static int oldTime = 0 ;

    if ( oldTime == 0 )
    {
        oldTime = currentTime ;
    }
            
    if ( currentTime - oldTime > 30000 )
    {
        oldTime = currentTime ;
    }
}

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

      // Serial.println(dateBuf);

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
    Serial.println(gpsLog.back().setDate);
    Serial.println(gpsLog.back().latitude);
    Serial.println(gpsLog.back().longitude);
    Serial.println(gpsLog.size()); // ログの件数表示
    // gpscounter++;
    // Serial.println(gpscounter);
  }
}



//---------------------------------------------------------------------
// Counter
// Count the number of loops in the main loop and turn on sensor data acquisition
// and BLE transmission at 1-second intervals
//---------------------------------------------------------------------
void loopCounter(){
  iLoop1s += 1;

  //--------------------
  // 1s period
  //--------------------
  if (iLoop1s >=  0){                 // 125ms x 8 = 1s   //現在はCTなし
    iLoop1s = 0;

    iSendCounter  += 1;
    if (iSendCounter >= SEND_INTERVAL){
      iSendCounter = 0;
      event1s = true;
    }
  }
}

//---------------------------------------------------------------------
// Sensor
// When sensor data acquisition is ON, data is acquired from each sensor
// Serial output of measured values and calculation results when console output is ON
//---------------------------------------------------------------------
void loopSensor(){
  double temp_mv;
    //-------------------------
    // LIS2DH
    // Data acquisition for 3-axis sensors
    //-------------------------
    accel.read();
    dataX_g = accel.x_g;    // X-axis
    dataY_g = accel.y_g;    // Y-axis
    dataZ_g = accel.z_g;    // Z-axis

    if(dataZ_g >= 1.0){
      dataZ_g = 1.00;
    } else if (dataZ_g <= -1.0){
      dataZ_g = -1.00;
    }

    dataTilt = acos(dataZ_g) / PI * 180;

    // Calculate the position of the dice.
    // For each eye, the sensor takes the following values
    //     X  Y  Z
    //  1  0  0  1
    //  2  0  1  0
    //  3 -1  0  0
    //  4  1  0  0
    //  5  0 -1  0
    //  6  0  0 -1
    if ((-0.5 <= dataX_g && dataX_g < 0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (0.5 <= dataZ_g)){
      dataPips = 1;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (0.5 <= dataY_g) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 2;
    }
    else if ((dataX_g < -0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 3;
    }
    else if ((0.5 <= dataX_g) && (-0.5 <= dataY_g && dataY_g < 0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 4;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (dataY_g < -0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 5;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (dataZ_g < -0.5)){
      dataPips = 6;
    }
//    else{
//      dataPips = 0;
//    }

    //-------------------------
    // HTS221
    // Temperature and humidity sensor data acquisition
    //-------------------------
    sensors_event_t humidity;
    sensors_event_t temp;

    hts.getEvent(&humidity, &temp);             // populate temp and humidity objects with fresh data
    dataTemp = temp.temperature;
    dataHumid =humidity.relative_humidity;

    //-------------------------
    // Two-point correction for temperature and humidity
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // Temperature correction
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // Humidity correction

    //-------------------------
    // OPT3001
    // Illuminance sensor data acquisition
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      dataLight = result.lux;
    }

  //-------------------------
  // ADC081C027（ADC)
  // Battery leaf battery voltage acquisition
  //-------------------------
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    // If the measured value is FF, the battery leaf is not connected.
    adcVal1 = adcVal2 = 0;
  }

  // Voltage calculation :　ADC　* ((Reference voltage(3.3V)/ ADC resolution(256)) * Divided voltage ratio(2)
  temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);
}

//---------------------------------------------------------------------
// Send sensor data
// Convert sensor data into a string to be sent to Central and send the data to BLE Leaf.
//---------------------------------------------------------------------
void bt_sendData(){
  float value;
  char temp[7], humid[7], light[7], tilt[7],battVolt[7], pips[7];
  char sendData[40];
  uint8 latLen = 9;
  uint8 lngLen = 10;
  uint8 sendLen = 14;

  //-------------------------
  // Convert sensor data to strings
  // dtostrf(Number to be converted, number of characters to be converted, number of decimal places, where to store the converted characters);
  // If the number of characters to be converted is set to -, the converted characters will be left-justified; if +, they will be right-justified.
  //-------------------------
  //-------------------------
  // Temperature (4Byte)
  //-------------------------
  value = dataTemp;
  if(value >= 100){
    value = 99.9;
  }
  else if(value <= -10){
    value = -9.9;
  }
  dtostrf(value,4,1,temp);

  //-------------------------
  // Humidity (4Byte)
  //-------------------------
  value = dataHumid;
  dtostrf(value,4,1,humid);

  //-------------------------
  // Ambient Light (5Byte)
  //-------------------------
  value = dataLight;
  if(value >= 100000){
    value = 99999;
  }
  dtostrf(value,5,0,light);

  //-------------------------
  // Tilt (4Byte)
  //-------------------------
  value = dataTilt;
  if(value < 3){
    value = 0;
  }
  dtostrf(value,4,0,tilt);

  //-------------------------
  // dice
  //-------------------------
 value = dataPips;
  if (value > 6){
    value = 0;
  }
  dtostrf(value,4,0,pips);
  
  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatt;

  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 2, battVolt);

  //-------------------------
  //無駄な末尾のデータを削除
  //-------------------------
  trim(temp);
  trim(humid);
  trim(light);
  trim(tilt);
  trim(battVolt);
  trim(pips);

  lcd.clear();

  if (dispLCD==1){
    switch (lcdSendCount){
      case 0:                 // BLE not connected
        lcd.print("Waiting");
        lcd.setCursor(0, 1);
        lcd.print("connect");
        break;
      case 1:                 // Tmp XX.X [degC]
        lcd.print("Temp");
        lcd.setCursor(0, 1);
        lcd.print( String(temp) +" C");
        break;
      case 2:                 // Hum xx.x [%]
        lcd.print("Humidity");
        lcd.setCursor(0, 1);
        lcd.print( String(humid) +" %");
        break;
      case 3:                 // Lum XXXXX [lx]
        lcd.print("Luminous");
        lcd.setCursor(0, 1);
        lcd.print( String(light) +" lx");
        break;
      case 4:                 // Ang XXXX [arc deg]
        lcd.print("Angle");
        lcd.setCursor(0, 1);
        lcd.print( String(tilt) +" deg");
        break;
      case 5:                 // Bat X.XX [V]
        lcd.print("Battery");
        lcd.setCursor(0, 1);
        lcd.print( String(battVolt) +" V");
        break;
      default:
        break;
    }
    if (lcdSendCount < 5){
      lcdSendCount++;
    }
    else{
        if( bBLEsendData == true ){                           // Start from 1 during BLE transmission.
          lcdSendCount = 1;
        }
        else{
          lcdSendCount = 0;
        }
    }
  }

  //-------------------------
  // BLE Send Data
  //-------------------------
  Serial.println("Hello");
  if( counter < 1 ){                                 // BLE transmission
    for (size_t i = 0; i < gpsLog.size(); i++){
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)gpsLog[i].setDate);
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog[i].setDate);
        // delay(1000);
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, latLen, (const uint8 *)gpsLog[i].latitude );
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog[i].latitude);
        // delay(1000);
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, lngLen, (const uint8 *)gpsLog[i].longitude );
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog[i].longitude);

    }


    counter++;
    Serial.println(counter);
  }
  if (counter == 1){
    strcpy(latestGPS.setDate, "00000000000000");
    strcpy(latestGPS.latitude, "00.000000\0");
    strcpy(latestGPS.longitude, "000.000000\0");

    trim(latestGPS.latitude);
    trim(latestGPS.longitude);
    trim(latestGPS.setDate);

    gpsLog.push_back(latestGPS);

    ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)gpsLog.back().setDate);
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog.back().setDate);
        // delay(1000);
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)gpsLog.back().latitude );
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog.back().latitude);
        // delay(1000);
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)gpsLog.back().longitude );
        while (ble112.checkActivity(1000));
        Serial.println(gpsLog.back().longitude);
        counter++;
        Serial.println(counter);
  }
}


//==============================================
// Interrupt
//==============================================

//----------------------------------------------
// Timer INT
// Timer interrupt function
//----------------------------------------------
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
    Serial.println("Setup BLE");
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

    Serial.println("BLE System Booted");
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
    Serial.println("Device Name Length:");
    ad_data[3] = (lenStr2 + 1);                                 // field length
    uint8 u8Index;
    for( u8Index=0; u8Index < lenStr2; u8Index++){
      ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
    }
    Serial.println("Device Name Set");
    /*   */
    stLen = (5 + lenStr2);

    //ble112.ble_cmd_le_gap_bt5_set_adv_data(0,SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);
    ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);
    Serial.println("BLE Advertising Data Setting...");
    while (ble112.checkActivity(1000));                         /* Receive check */
    delay(20);
    Serial.println("BLE Advertising Data Set");
    /* interval_min :   40ms( =   64 x 0.625ms ) */
    //ble112.ble_cmd_le_gap_bt5_set_adv_parameters( 0, 64, 1600, 7, 0 );/* [BGLib] <handle> <interval_min> <interval_max> <channel_map> <report_scan>*/
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */

    while (ble112.checkActivity(1000));                         /* [BGLIB] Receive check */
    Serial.println("BLE Advertising Parameters Set");
    /* start */
//    ble112.ble_cmd_le_gap_bt5_set_mode(0,LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE,0,2);
//    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);     // index = 0
    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);                // index = 0
    while (ble112.checkActivity(1000));                         /* Receive check */
    /*  */
    Serial.println("BLE finisshed");
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
