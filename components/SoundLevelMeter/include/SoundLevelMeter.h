#pragma once
#include "FilterInterface.h"
#include "TimeWeighting.h"
#include <memory>

namespace slm {

enum class FrequencyWeighting : uint32_t { A, C, Z };
enum class TimeWeighting : uint32_t { FAST, SLOW, IMPULSE, };

struct MeterResults {
  float peak;
  float leq;
  float spl;
};

struct SLMConfig {
  FrequencyWeighting fW = FrequencyWeighting::A;
  TimeWeighting tW = TimeWeighting::FAST;
};

class SoundLevelMeter {
public:
  SoundLevelMeter(const SLMConfig& cfg);
  SoundLevelMeter(const SLMConfig& cfg, const float& calibrationFactor);
  ~SoundLevelMeter() = default;

  void reset(const SLMConfig &cfg);
  MeterResults process(const float &sample);
  float calibrate(const float& sample);

private:
  std::unique_ptr<Filtering::FilterInterface> mFreqWeighting;
  Filtering::TimeWeighting mTimeWeighting;
  float mPeak;
  float mCalibratedLevel;
};

} // namespace slm
