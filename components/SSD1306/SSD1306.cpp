#include "SSD1306.h"
using namespace Display;

SSD1306::SSD1306(const SSD1306Config& config, IDisplayBus& bus)
    : mConfig(config), mBus(bus)
{
}

bool SSD1306::initialize()
{
  if (mConfig.hwResetFn) {
    if (!mConfig.hwResetFn(mConfig.addrOrCs)) {
      return false;
    }
  }

  uint8_t multiplex = static_cast<uint8_t>(mConfig.height - 1);
uint8_t com_pins = (mConfig.height == 32) ? 0x02 : 0x12;
  uint8_t init_sequence[] = {
  0xAE, // Display OFF
  0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
  0xA8, multiplex, // Set multiplex ratio
  0xD3, 0x00, // Set display offset
  0x40, // Set display start line
  0x8D, 0x14, // Charge pump setting (enable)
  0x20, 0x00, // Memory addressing mode (horizontal)
  0xA1, // Segment remap
  0xC8, // COM output scan direction
  0xDA, com_pins, // COM pins hardware configuration
  0x81, 0xCF, // Contrast control
  0xD9, 0xF1, // Pre-charge period
  0xDB, 0x40, // VCOMH deselect level
  0xA4, // Entire display ON (resume)
  0xA6, // Normal display
  0xAE  // Display OFF
  };
  if (!Command(init_sequence, sizeof(init_sequence))) {
    return false;
  }

  uint8_t col_addr[] = {0x21, 0x00, static_cast<uint8_t>(mConfig.width - 1)};
  if (!Command(col_addr, sizeof(col_addr))) return false;
  uint8_t page_addr[] = {0x22, 0x00, static_cast<uint8_t>((mConfig.height / 8) - 1)};
  if (!Command(page_addr, sizeof(page_addr))) return false;
  uint8_t displayValue = 0x00;
  for (size_t i = 0; i < (mConfig.width * mConfig.height / 8); ++i) {
    if (!Data(&displayValue, 1)) {
      return false;
    }
  }

  uint8_t displayOn = {0xAF}; // Display ON
  return Command(&displayOn, 1);
}

bool SSD1306::Command(const uint8_t* commands, size_t len)
{
  return mBus.write(mConfig.addrOrCs, commands, len, true);
}

bool SSD1306::Data(const uint8_t* data, size_t len)
{
  return mBus.write(mConfig.addrOrCs, data, len, false);
}

bool SSD1306::writeData(const uint8_t* data, size_t len)
{
  uint8_t addr[] = {0x21, 0x00, static_cast<uint8_t>(mConfig.width - 1), 0x22, 0x00, static_cast<uint8_t>((mConfig.height / 8) - 1)};
  if (!Command(addr, sizeof(addr))) return false;
  return Data(data, len);
}

