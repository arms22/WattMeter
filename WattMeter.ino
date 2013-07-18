#include <LiquidCrystal.h>
#include <BigFont.h>
#include <Time.h>
#include <Bounce.h>
#include "Config.h"
#include "WattLib.h"

#define WATT_DEBUG 0

// ピン定義  
const int viasPin = 0;
const int ct1Pin = 1;
const int vt1Pin = 2;
const int ct2Pin = 3;
const int vt2Pin = 4;
const int displayBtnPin = 9;

// キャラクタLCDクラス（RS=2, R/W=3, E=4, D4=5, D5=6, D4=7, D3=8)
LiquidCrystal lcdc(2,3,4,5,6,7,8);

// ビッグフォントクラス
BigFont bigf;

// ボタン入力
Bounce displayButton(displayBtnPin, 20);

// 表示モード
enum {
  REALTIME,
  TOTAL,
  HISTORY,
};
byte displayMode = REALTIME;

// 消費電力
float wattSum;
byte wattSumCount;

// 1日の消費電力
float wattDay;

// 過去の消費電力
#define NUM_HISTORY (1 + 8)
float wattHistory[NUM_HISTORY];
byte wattHistoryWp;

int lastMinute;
int lastDay;

void setup()
{
#if WATT_DEBUG
  Serial.begin(115200);
#endif
  pinMode(displayBtnPin, INPUT_PULLUP);

  lcdc.begin(20,4);
  lcdc.print(F("Watt Meter"));
  bigf.attach(&lcdc);

  wattSum = 0;
  wattSumCount = 0;
  wattDay = 0;

  for(byte i=0; i<NUM_HISTORY; i++)
    wattHistory[i] = i * 60;
  wattHistoryWp = 0;

  lastMinute = 0;
  lastDay = 1;

  setTime(0, 0, 0, 1, 1, 2013);
}

void loop()
{
  float wattTotal;
  float wattNow;

  // リアルタイム表示
  //   電圧1  電流1  電力1
  //   電圧2  電流2  電力2
  // 今日の消費電力(kWh)
  // 過去7日間の消費電力(kWh)
  //
  // ボタン押し下げで表示を切り替える
  //

  WattResult result1 = calcWattVIIV(viasPin, vt1Pin, ct1Pin, 50, DEFAULT_AREF, DEFAULT_kVT1, DEFAULT_kCT);
  WattResult result2 = calcWattVIIV(viasPin, vt2Pin, ct2Pin, 50, DEFAULT_AREF, DEFAULT_kVT2, DEFAULT_kCT);

#if WATT_DEBUG
  Serial.print("samples1 ");
  Serial.print(result1.samples);
  Serial.print(" vrms ");
  Serial.print(result1.vrms);
  Serial.print(" irms ");
  Serial.print(result1.irms);
  Serial.print(" watt ");
  Serial.println(result1.realPower);
  Serial.print("apparentPower ");
  Serial.print(result1.apparentPower);
  Serial.print(" powerFactor ");
  Serial.println(result1.powerFactor);

  Serial.print("samples2 ");
  Serial.print(result2.samples);
  Serial.print(" vrms ");
  Serial.print(result2.vrms);
  Serial.print(" irms ");
  Serial.print(result2.irms);
  Serial.print(" watt ");
  Serial.println(result2.realPower);
  Serial.print("apparentPower ");
  Serial.print(result2.apparentPower);
  Serial.print(" powerFactor ");
  Serial.println(result2.powerFactor);

  Serial.print(year());
  Serial.print('/');
  Serial.print(month());
  Serial.print('/');
  Serial.print(day());
  Serial.print(' ');  
  Serial.print(hour());
  Serial.print(':');
  Serial.print(minute());
  Serial.print(':');
  Serial.println(second());
#endif

  wattNow = result1.realPower + result2.realPower;
  wattSum += wattNow;
  wattSumCount++;

  if(lastMinute != minute()){
    wattDay += (wattSum / wattSumCount);
    wattHistory[wattHistoryWp] = wattDay;
    wattSum = 0;
    wattSumCount = 0;
    lastMinute = minute();
  }

  if(lastDay != day()){
    wattHistory[wattHistoryWp] = wattDay;
    wattDay = 0;
    wattHistoryWp = (wattHistoryWp + 1) % NUM_HISTORY;
    if(wattHistoryWp >= NUM_HISTORY){
      wattHistoryWp = 0;
    }
    lastDay = day();
  }

  if(displayButton.update()){
    if(displayButton.read() == LOW){
      displayMode++;
      if(displayMode > HISTORY){
        displayMode = REALTIME;
      }
    }
  }

  lcdc.clear();

  switch(displayMode){
  case REALTIME:
    lcdc.print(result1.vrms, 1);
    lcdc.print(F("V "));
    lcdc.print(result1.irms, 1);
    lcdc.print(F("A "));
    lcdc.print(result1.realPower, 1);
    lcdc.print(F("W "));

    lcdc.setCursor(0, 1);
    lcdc.print(result2.vrms, 1);
    lcdc.print(F("V "));
    lcdc.print(result2.irms, 1);
    lcdc.print(F("A "));
    lcdc.print(result2.realPower, 1);
    lcdc.print(F("W "));

    showWattBigFont(wattNow, "");
    break;
  case TOTAL:
    lcdc.setCursor(0, 0);
    lcdc.print(F("      Total         "));
    lcdc.setCursor(0, 1);
    lcdc.print(F(" Power Consumption  "));

    showWattBigFont(wattDay / 60.0, "h");
    break;
  case HISTORY:
    showWattHistory();
    break;
  }

}

void showWattBigFont(float watt, char *postsuffix){
  bigf.setCursor(0, 2);
  if(watt >= 1100.0){
    float kwatt = watt / 1000.0;
    if(watt < 9995.0){
      bigf.print(kwatt, 2);
      lcdc.print(F("kW"));
    }
    else if(watt < 99950.0){
      bigf.print(kwatt, 1);
      lcdc.print(F("kW"));
    }
    else{
      bigf.print(kwatt, 0);
      lcdc.print(F("kW"));
    }
  }
  else{
    bigf.print((int)watt);
    lcdc.print(F("W"));
  }
  lcdc.print(postsuffix);
}

void showWattHistory(void){
  byte i;
  byte idx = wattHistoryWp;
  for(i=0; i<8; i++){
    byte col, row;
    idx = (NUM_HISTORY - 1 + idx) % NUM_HISTORY;
    col = (i / 4) * 10;
    row = (i % 4);
    lcdc.setCursor(col, row);
    lcdc.print(i);
    lcdc.print(F(":"));
    lcdc.print((int)(wattHistory[idx] / 60.0));
    lcdc.print(F("Wh"));
  }
}

