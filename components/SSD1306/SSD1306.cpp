#include "SSD1306.h"
using namespace Display;

SSD1306::SSD1306(const SSD1306Config& config, IDisplayBus& bus)
    : mConfig(config), mBus(bus)
{
}

bool SSD1306::initialize()
{
  if (mConfig.reset_func) {
    if (!mConfig.reset_func(mConfig.addrOrCs)) {
      return false;
    }
  }

  uint8_t multiplex = static_cast<uint8_t>(mConfig.height - 1);
  uint8_t com_pins = 0x02; // Sequential COM pin config, disable left/right remap
  if (mConfig.height > 32) {
    com_pins |= 0x10; // Enable alternative COM pin config for >32 rows
  }
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
  0xAF  // Display ON
  };

  return Command(init_sequence, sizeof(init_sequence));
}

bool SSD1306::Command(const uint8_t* commands, size_t len)
{
  return mBus.write(mConfig.addrOrCs, commands, len, true);
}

bool SSD1306::Data(const uint8_t* data, size_t len)
{
  return mBus.write(mConfig.addrOrCs, data, len, false);
}

bool SSD1306::write(const uint8_t* data, size_t len, bool isCommand)
{
  uint8_t col_addr[] = {0x21, 0x00, static_cast<uint8_t>(mConfig.width - 1)};
  if (!Command(col_addr, sizeof(col_addr))) return false;
  uint8_t page_addr[] = {0x22, 0x00, static_cast<uint8_t>((mConfig.height / 8) - 1)};
  if (!Command(page_addr, sizeof(page_addr))) return false;
  return mBus.write(mConfig.addrOrCs, data, len, isCommand);
}

