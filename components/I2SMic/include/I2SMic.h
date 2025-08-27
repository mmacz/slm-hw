#pragma once

#include <cstdint>
#include <cstddef>

namespace Microphone {
struct I2SMicConfig {
  uint32_t sampleRate;
  bool isStereo;
  bool useDMA;

  I2SMicConfig(uint32_t fs, bool stereo = false, bool dma = false)
  : sampleRate(fs)
  , isStereo(stereo)
  , useDMA(dma)
  {}
};

template <typename T>
class I2SInterface {
public:
  virtual ~I2SInterface() = default;
  virtual bool initialize(const I2SMicConfig& config) = 0;
  virtual bool readSamples() = 0;
  virtual size_t available() const = 0;
  virtual T getSample() = 0;
};

template <typename T>
class I2SMic {
public:
  I2SMic(const I2SMicConfig& config, I2SInterface<T>& i2s)
  : mI2SInterface(i2s)
  , mConfig(config)
  {}

  bool initialize() {
    return mI2SInterface.initialize(mConfig);
  }

  T getSample() {
    return mI2SInterface.getSample();
  }

  bool readSamples() {
    return mI2SInterface.readSamples();
  }

  size_t available() const {
    return mI2SInterface.available();
  }


private:
  I2SInterface<T>& mI2SInterface;
  I2SMicConfig mConfig;
};

} // namespace Microphone

