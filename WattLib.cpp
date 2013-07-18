#include <Arduino.h>
#include "WattLib.h"

#include <avr/sleep.h>

ISR(ADC_vect){
}

int analogReadEx(uint8_t pin)
{
  uint8_t low, high;

  ADCSRA = 0x8E; // ADEN=1, ADIE = 1, ADPS[2-0] = 64

  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = (1 << 6) | pin & 0x07;

  // start the conversion
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();
  do{
    cli();
    if(bit_is_set(ADCSRA, ADSC)){
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
    }
    else{
      sei();
      break;
    }
  }
  while(1);

  low  = ADCL;
  high = ADCH;

  // combine the two bytes
  return (high << 8) | low;
}

//#define ANALOGREAD  analogRead
#define ANALOGREAD  analogReadEx

WattResult calcWattVIIV(int pinBias, int pinV, int pinI, int crossings, float Aref, float kVT, float kCT)
{
  WattResult result;
  float sampleV, sqV, sumV, sampleI, sqI, sumI, instP, sumP;
  unsigned long samples;
  int bias, v1, v2, i1, i2, crossCount;
  uint8_t checkVCross, lastVCross;

  crossCount = 0;
  samples = 0;
  sumV = sumI = sumP = 0;

  // read DC offset.
  bias = ANALOGREAD(pinBias);

  //
  // 1) Waits for the waveform to be close to 'zero' (500 adc) part in sin curve.
  //
  lastVCross = checkVCross = (ANALOGREAD(pinV) > bias);
  while(lastVCross == checkVCross) {
    lastVCross = checkVCross;
    checkVCross = (ANALOGREAD(pinV) > bias);
  }

  //
  // 2) Main measurment loop
  //

  while (crossCount < crossings) {

#if 1
    int sample[9];

    sample[0] = ANALOGREAD( pinV );
    sample[1] = ANALOGREAD( pinI );
    sample[2] = ANALOGREAD( pinBias );
    sample[3] = ANALOGREAD( pinI );
    sample[4] = ANALOGREAD( pinV );
    sample[5] = ANALOGREAD( pinI );
    sample[6] = ANALOGREAD( pinBias );
    //    sample[7] = ANALOGREAD( pinI );
    //    sample[8] = ANALOGREAD( pinV );

    bias = sample[2];
    v2 = (sample[4] + sample[0]) >> 1;
    sampleV = v2 - bias;
    sampleI = ((sample[3] + sample[1]) >> 1) - bias;

    sqV = sampleV * sampleV;
    sumV += sqV;
    sqI = sampleI * sampleI;
    sumI += sqI;
    instP = sampleV * sampleI;
    sumP += instP;
    samples++;

    bias = (sample[6] + sample[2]) >> 1;
    v2 = sample[4];
    sampleV = v2 - bias;
    sampleI = ((sample[5] + sample[3]) >> 1) - bias;

    sqV = sampleV * sampleV;
    sumV += sqV;

    sqI = sampleI * sampleI;
    sumI += sqI;

    instP = sampleV * sampleI;
    sumP += instP;
    samples++;

    //    bias = sample[6];
    //    v2 = (sample[8] + sample[4]) >> 1;
    //    sampleV = v2 - bias;
    //    sampleI = ((sample[7] + sample[5]) >> 1) - bias;
    //    
    //    sqV = sampleV * sampleV;
    //    sumV += sqV;
    //    sqI = sampleI * sampleI;
    //    sumI += sqI;
    //    instP = sampleV * sampleI;
    //    sumP += instP;
    //    samples++;
#else
    v1 = ANALOGREAD( pinV );
    i1 = ANALOGREAD( pinI );
    bias = ANALOGREAD(pinBias);
    i2 = ANALOGREAD( pinI );
    v2 = ANALOGREAD( pinV );

    sampleV = ((v1 + v2) >> 1) - bias;
    sampleI = ((i1 + i2) >> 1) - bias;

    sqV = sampleV * sampleV;
    sumV += sqV;

    sqI = sampleI * sampleI;
    sumI += sqI;

    instP = sampleV * sampleI;
    sumP += instP;

    samples++;
#endif

    lastVCross = checkVCross;
    checkVCross = (v2 > bias);
    if(lastVCross != checkVCross){
      crossCount++;
    }
  }

  //
  // 3) Post loop calculations
  //
  // Calculation of the root of the mean of the voltage and current squared (rms)
  // Calibration coeficients applied. 
  //

  float V_RATIO = kVT * (Aref / 1024.0);
  result.vrms = V_RATIO * sqrt((float)sumV / samples);

  float I_RATIO = kCT * (Aref / 1024.0);
  result.irms = I_RATIO * sqrt((float)sumI / samples);

  // Calculation power values
  result.realPower = (V_RATIO * I_RATIO * (float)sumP) / samples;
  result.apparentPower = result.vrms * result.irms;
  result.powerFactor = result.realPower / result.apparentPower;
  result.samples = samples;

  return result;
}

WattResult calcWattVIVI(int pinBias, int pinV, int pinI, int crossings, float Aref, float kVT, float kCT)
{
  WattResult result;
  float sampleV, sqV, sumV, sampleI, sqI, sumI, instP, sumP;
  unsigned long samples;
  int bias, crossCount, sample[3], vv;
  uint8_t checkVCross, lastVCross;

  // read DC offset.
  bias = ANALOGREAD(pinBias);

  //
  // 1) Waits for the waveform to be close to 'zero' (500 adc) part in sin curve.
  //
  lastVCross = checkVCross = (ANALOGREAD(pinV) > bias);
  while(lastVCross == checkVCross) {
    lastVCross = checkVCross;
    checkVCross = (ANALOGREAD(pinV) > bias);
  }

  //
  // 2) Main measurment loop
  //
  crossCount = 0;
  samples = 0;
  sumV = sumI = sumP = 0;

  bias = ANALOGREAD(pinBias);

  sample[1] = ANALOGREAD( pinV );
  delayMicroseconds(74);

  sample[2] = ANALOGREAD( pinI );
  delayMicroseconds(74);

  while (crossCount < crossings) {

    sample[0] = sample[1];
    sample[1] = sample[2];

    if((samples & 1) == 0){
      sample[2] = ANALOGREAD( pinV );
      sampleV = ((sample[2] + sample[0]) >> 1) - bias;
      sampleI = sample[1] - bias;
      vv = sample[2];
    }
    else{
      sample[2] = ANALOGREAD( pinI );
      sampleI = ((sample[2] + sample[0]) >> 1) - bias;
      sampleV = sample[1] - bias;
      vv = sample[1];
    }

    sqV = sampleV * sampleV;
    sqI = sampleI * sampleI;
    instP = sampleV * sampleI;

    sumV += sqV;
    sumI += sqI;
    sumP += instP;

    lastVCross = checkVCross;
    checkVCross = (vv > bias);
    if(lastVCross != checkVCross){
      crossCount++;
    }

    samples++;
  }

  //
  // 3) Post loop calculations
  //
  // Calculation of the root of the mean of the voltage and current squared (rms)
  // Calibration coeficients applied. 
  //

  float V_RATIO = kVT * (Aref / 1024.0);
  result.vrms = V_RATIO * sqrt((float)sumV / samples);

  float I_RATIO = kCT * (Aref / 1024.0);
  result.irms = I_RATIO * sqrt((float)sumI / samples);

  // Calculation power values
  result.realPower = (V_RATIO * I_RATIO * sumP) / samples;
  result.apparentPower = result.vrms * result.irms;
  result.powerFactor = result.realPower / result.apparentPower;
  result.samples = samples;

  return result;
}



