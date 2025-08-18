#pragma once

#include <cstdint>
#include <memory.h>

namespace gfx {
class GFX {
public:
  GFX(uint16_t width, uint16_t height, uint32_t bufferSize, uint8_t* buffer);
  ~GFX() = default;

  void fillScreen(uint8_t color);
  bool setPixel(uint16_t x, uint16_t y, uint8_t color);
  bool drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color);
  bool drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);
  bool fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);
  bool drawChar(uint16_t x, uint16_t y, char c, uint8_t color, const uint8_t* pFont);
  bool drawString(uint16_t x, uint16_t y, const char *str, uint8_t color, const uint8_t* pFont);

  uint16_t width() const;
  uint16_t height() const;
  uint32_t bufferSize() const;
  const uint8_t* buffer() const;

private:
  uint16_t mWidth;
  uint16_t mHeight;
  uint8_t* mBuffer;
  uint32_t mBufferSize;
};
}

