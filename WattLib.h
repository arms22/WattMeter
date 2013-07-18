#ifndef _WATT_LIB_
#define _WATT_LIB_

// サンプル結果用構造体
typedef struct {
  float vrms;
  float irms;
  float realPower;
  float apparentPower;
  float powerFactor;
  unsigned long samples;
} 
WattResult;

WattResult calcWattVIIV(int pinVias, int pinV, int pinI, int crossings, float Aref, float kVT, float kCT);
WattResult calcWattVIVI(int pinVias, int pinV, int pinI, int crossings, float Aref, float kVT, float kCT);

#endif

