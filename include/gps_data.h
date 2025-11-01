#ifndef GPS_DATA_H
#define GPS_DATA_H

#include <stdint.h>
#include <vector>

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
};

// グローバル変数は .cpp 側で定義する
extern GPSData latestGPS;
extern std::vector<GPSData> gpsLog;

#endif // GPS_DATA_H
