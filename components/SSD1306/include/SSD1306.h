#pragma once

#include <cstdint>
#include <cstddef>

namespace Display {
  class IDisplayBus {
  public:
    virtual ~IDisplayBus() = default;
    virtual bool write(uint8_t addrOrCS, const uint8_t* data, size_t len, bool isCommand) = 0;
  };

  using HWDisplayResetFn = bool (*)(uint8_t pin);

  struct SSD1306Config {
    uint8_t addrOrCs;
    uint16_t width;
    uint16_t height;
    HWDisplayResetFn reset_func = nullptr;

    SSD1306Config(uint8_t addrOrCs, uint16_t w, uint16_t h, HWDisplayResetFn hwReset = nullptr)
        : addrOrCs(addrOrCs), width(w), height(h), reset_func(hwReset) {};
  };

  class SSD1306 {
  public:
    SSD1306(const SSD1306Config& config, IDisplayBus& bus);

    bool initialize();
    uint16_t width() const { return mConfig.width; }
    uint16_t height() const { return mConfig.height; }
    uint8_t address() const { return mConfig.addrOrCs; }
    bool write(const uint8_t* data, size_t len, bool isCommand = false);

  private:
    bool Command(const uint8_t* commands, size_t len);
    bool Data(const uint8_t* data, size_t len);
    SSD1306Config mConfig;
    IDisplayBus& mBus;
  };

} // namespace Display
