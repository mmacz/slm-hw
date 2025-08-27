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

bool GFX::setPixel(uint16_t x, uint16_t y, uint8_t color) {
  if (!mBuffer || x >= mWidth || y >= mHeight) return false;
  uint32_t index = x + (y >> 3) * mWidth;
  uint8_t bit = 1 << (y & 7);
  if (color)
    mBuffer[index] |= bit;
  else
    mBuffer[index] &= ~bit;
  return true;
}

bool GFX::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color) {
  bool ok = true;
  int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true) {
    ok &= setPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return ok;
}

bool GFX::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color) {
  bool ok = true;
  ok &= drawLine(x, y, x + w - 1, y, color);
  ok &= drawLine(x, y + h - 1, x + w - 1, y + h - 1, color);
  ok &= drawLine(x, y, x, y + h - 1, color);
  ok &= drawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
  return ok;
}

bool GFX::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color) {
  bool ok = true;
  for (uint16_t i = 0; i < h; ++i) {
    ok &= drawLine(x, y + i, x + w - 1, y + i, color);
  }
  return ok;
}

bool GFX::drawChar(uint16_t x, uint16_t y, char chr, uint8_t color, const font_desc_t* pFont) {
  if (chr < pFont->first_char || chr > pFont->last_char) {
    return false;
  }
  bool ok = true;
  const uint32_t* pFontData = pFont->font;
  uint16_t chrIdx = ((uint16_t)(chr - pFont->first_char)) * pFont->width;
  for (uint16_t col = 0; col < pFont->width; ++col) {
    uint32_t bits = pFontData[chrIdx + col];
    for (uint16_t row = 0; row < pFont->height; ++row) {
      if (bits & (1 << row)) {
        ok &= setPixel(x + col, y + row, color);
      } else {
        ok &= setPixel(x + col, y + row, !color);
      }
    }
  }
  return ok;
}

bool GFX::drawString(uint16_t x, uint16_t y, const char *str, uint8_t color, const font_desc_t* pFont) {
  bool ok = true;
  uint16_t xx = x;
  const char* pStr = str;
  while (*pStr) {
    ok &= drawChar(xx, y, *pStr, color, pFont);
    xx += pFont->width;
    ++pStr;
  }
  return ok;
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

