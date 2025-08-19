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
  ~SoundLevelMeter() = default;

  void reset(const SLMConfig &cfg);
  MeterResults process(const float &sample);
  float calibrate(const float& sample, float alpha = .1f);

private:
  std::unique_ptr<Filtering::FilterInterface> mFreqWeighting;
  Filtering::TimeWeighting mTimeWeighting;
  double mLeqEnergySum;
  uint64_t mLeqSamples;
  float mPeakAbs;
  float mSampleRef;
  float mRmsRef;
  float mCalGain;
  float mCalibratedLevel;
};

} // namespace slm
