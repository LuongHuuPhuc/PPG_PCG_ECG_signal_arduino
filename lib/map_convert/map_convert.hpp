/**
 * @author Luong Huu Phuc
 * @date 2025/03/13
 */
#ifndef MAP_CONVERT_HPP_
#define MAP_CONVERT_HPP_

#include <stdint.h>
#include <Adafruit_GFX.h>

typedef struct {
  int x_pos;
  int y_pos;
  int last_x;
  int last_y;
} x_y_position_t;

/**
 * @brief Ham de chuan hoa du lieu ve dang pixel
 */
long map_convert(long input, long in_min, long in_max, long out_min, long out_max);

#endif