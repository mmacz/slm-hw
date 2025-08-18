#pragma once

#include <cstdint>
#include <cstddef>

namespace Microphone {
struct INMP441Config {
  uint32_t sampleRate;
  bool isStereo;
  bool useDMA;

  INMP441Config(uint32_t fs, bool stereo = false, bool dma = false)
  : sampleRate(fs)
  , isStereo(stereo)
  , useDMA(dma)
  {}
};

template <typename T>
class I2SInterface {
public:
  virtual ~I2SInterface() = default;
  virtual bool initialize(const INMP441Config& config) = 0;
  virtual bool writeSamples() = 0;
  virtual T readSample() = 0;
};

template <typename T>
class INMP441 {
public:
  INMP441(const INMP441Config& config, I2SInterface<T>& i2s)
  : mI2SInterface(i2s)
  , mConfig(config)
  {}

  bool initialize() {
    return mI2SInterface.initialize(mConfig);
  }

  T readSample() {
    return mI2SInterface.readSample();
  }

  bool writeSamples() {
    return mI2SInterface.writeSamples();
  }


private:
  I2SInterface<T>& mI2SInterface;
  INMP441Config mConfig;
};

} // namespace Microphone

