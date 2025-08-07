#pragma once
#include "FilterInterface.h"
#include "TimeWeighting.h"
#include <memory>

namespace slm {

enum class FrequencyWeighting { A, C, Z };

enum class TimeWeighting {
  FAST,
  SLOW,
  IMPULSE,
};

struct MeterResults {
  float peak;
  float leq;
  float spl;
};

struct SLMConfig {
  FrequencyWeighting fW;
  TimeWeighting tW;
};

class SoundLevelMeter {
public:
  SoundLevelMeter(const SLMConfig &cfg);
  ~SoundLevelMeter() = default;

  void reset(const SLMConfig &cfg);
  MeterResults process(const float &sample);
  const float calibrate(const float& sample);

private:
  std::unique_ptr<Filtering::FilterInterface> mFreqWeighting;
  Filtering::TimeWeighting mTimeWeighting;
  float mPeak;
  float mCalibratedLevel;
};

} // namespace slm
