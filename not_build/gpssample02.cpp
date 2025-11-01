#include <SoftwareSerial.h>
void clearBufferArray()  ;
SoftwareSerial SoftSerial(A1, A2);
unsigned char buffer[64];                   // シリアルポート経由で受信するデータ用のバッファ配列
int count=0;                                // バッファ配列のカウンター
void setup()
{
    SoftSerial.begin(9600);                 // SoftSerialのボーレート
    Serial.begin(9600);                     // Arduinoのシリアルポートのボーレート
}

void loop()
{
    if (SoftSerial.available())                     // ソフトウェアシリアルポートからデータが来ている場合
    {
        while(SoftSerial.available())               // データを文字配列に読み込む
        {
            buffer[count++]=SoftSerial.read();      // 配列にデータを書き込む
            if(count == 64)break;
        }
        Serial.write(buffer,count);                 // データ送信が終了していない場合、バッファをハードウェアシリアルポートに書き込む
        clearBufferArray();                         // clearBufferArray関数を呼び出して配列内のデータをクリアする
        count = 0;                                  // whileループのカウンターをゼロに設定
    }
    if (Serial.available())                 // ハードウェアシリアルポートにデータがある場合（PCまたはノートブックからデータが来ている場合）
    SoftSerial.write(Serial.read());        // SoftSerialシールドに書き込む
}


void clearBufferArray()                     // バッファ配列をクリアする関数
{
    for (int i=0; i<count;i++)
    {
        buffer[i]=NULL;
    }                      // 配列のすべてのインデックスをNULLでクリア
}