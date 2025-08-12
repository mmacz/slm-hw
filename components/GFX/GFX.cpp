#include "GFX.h"
using namespace gfx;

GFX::GFX(uint16_t width, uint16_t height, uint32_t bufferSize, uint8_t* buffer)
    : mWidth(width), mHeight(height), mBuffer(buffer), mBufferSize(bufferSize) {
}

void GFX::fillScreen(uint8_t color) {
  if (!mBuffer) return;
  for (uint32_t i = 0; i < mBufferSize; ++i) {
    mBuffer[i] = color;
  }
}

uint16_t GFX::width() const {
  return mWidth;
}

uint16_t GFX::height() const {
  return mHeight;
}

uint32_t GFX::bufferSize() const {
  return mBufferSize;
}

const uint8_t* GFX::buffer() const {
  return mBuffer;
}

