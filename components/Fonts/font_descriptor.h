#pragma once

#include <stdint.h>

typedef struct font_desc_s {
  uint8_t monospace;
  uint8_t width;
  uint8_t height;
  uint8_t first_char;
  uint8_t last_char;
  const uint16_t* font;
} font_desc_t;

