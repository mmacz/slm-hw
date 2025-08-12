#pragma once

#include <cstdint>
#include <memory.h>

namespace gfx {
class GFX {
public:
  GFX(uint16_t width, uint16_t height, uint32_t bufferSize, uint8_t* buffer);
  ~GFX() = default;

  void fillScreen(uint8_t color);

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

