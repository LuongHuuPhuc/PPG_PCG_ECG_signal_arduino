#include "Arduino.h"
#include "map_convert.hpp"
#include "esp32-hal-log.h"


long map_convert(long input, long in_min, long in_max, long out_min, long out_max){
  const long run = in_max - in_min;
  if(run == 0){
    log_e("map_covert(): Invalid input range, min = max ?");
    return(-1);
  }
  const long rise = out_max - out_min;
  const long delta = input - in_min;
  return (delta * rise) / run + out_min;
}