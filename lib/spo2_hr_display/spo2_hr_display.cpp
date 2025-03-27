#include "spo2_hr_display.hpp"
#include <Arduino.h>

double ESpO2 = 60.0;
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
int Num = 30;

void low_pass_for_IR_RED(uint32_t red, uint32_t ir, spo2_hr_display_t *param, int sample_taken){
  param->fir = (float)ir;
  param->fred = (float)red;
  param->aveir = param->aveir * frate + (float)ir * (1.0 - frate); //average IR level by low pass filter
  param->avered = param->avered * frate + (float)red * (1.0 - frate);//average red level by low pass filter
  param->sumirrms += (param->fir - param->aveir) * (param->fir - param->aveir);//square sum of alternate component of IR level
  param->sumredrms += (param->fred - param->avered) * (param->fred - param->avered); //square sum of alternate component of red level

  if((sample_taken % Num) == 0){
    float R = (sqrt(param->sumirrms) / param->aveir) / (sqrt(param->sumredrms) / param->avered);
    param->SpO2 = -23.3* (R - 0.4) + 120; //Cong thuc SpO2 chuan hoa
    ESpO2 = FSpO2 * ESpO2  + (1.0 - FSpO2) * param->SpO2; //Low pass filter
    if(ESpO2 <= MINIMUM_SPO2){
      ESpO2 = MINIMUM_SPO2;
    }
    if(ESpO2 > 100){
      ESpO2 = 99.9;
    }
    param->sumredrms = 0.0;
    param->sumirrms = 0.0;
    param->SpO2 = 0;
    sample_taken = 0;
  }
}
  
void heart_rate_calculate(spo2_hr_display_t *param){
  long delta = millis() - lastBeat;
  lastBeat = millis();
  param->beatsPerMinute = 60 / (delta / 1000.0);
  if(param->beatsPerMinute < 255 && param->beatsPerMinute > 20){
    rates[rateSpot++] = (byte)param->beatsPerMinute;
    rateSpot %= RATE_SIZE;
    param->beatAvg = 0;
    for(int i = 0; i < RATE_SIZE; i++){
      param->beatAvg += rates[i];
    }
    param->beatAvg /= RATE_SIZE;
  }
}