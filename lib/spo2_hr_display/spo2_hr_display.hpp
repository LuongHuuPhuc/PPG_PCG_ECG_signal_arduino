/**
 * @author Luong Huu Phuc 
 * @date 2025/03/13
 */

#ifndef SPO2_HR_DISPLAY_H_
#define SPO2_HR_DISPLAY_H_

#include <stdint.h>
#include <stdio.h>
#include "Arduino.h"

#define FINGER_ON 7000   
#define MINIMUM_SPO2 60.0 
#define BUZZER_PIN 13
#define RATE_SIZE 4

//Cac bien nay se duoc dinh nghia o ngoai
extern double frate;  //Low pass filter cho IR/RED de tinh gia tri AC 
extern double FSpO2; //Filter factor cho spo2 da duoc tinh
extern double ESpO2;
extern long lastBeat;  
extern int Num;
extern byte rateSpot;
static byte rates[RATE_SIZE];

typedef struct {
  double avered; //Trung binh tin hieu do
  double aveir; //Trung binh tin hieu IR
  double sumirrms; 
  double sumredrms;
  double fred;
  double fir;
  double SpO2;
  float beatsPerMinute;
  int beatAvg;
} spo2_hr_display_t;

void low_pass_for_IR_RED(uint32_t red, uint32_t ir, spo2_hr_display_t *param, int sample_taken);

void heart_rate_calculate(spo2_hr_display_t *param);

#endif 