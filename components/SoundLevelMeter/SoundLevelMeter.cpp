#include "SoundLevelMeter.h"
#include "AWeighting.h"
#include "CWeighting.h"
#include "FilterCoeffs.h"

#include <cstdint>
#include <cmath>

namespace slm {

uint32_t GetTimeWeightingTimeMs(TimeWeighting tW) {
  switch (tW) {
  case TimeWeighting::FAST:
    return 125;
  case TimeWeighting::SLOW:
    return 1000;
  case TimeWeighting::IMPULSE:
    return 35;
  default:
    while (1) ;
  }
};

constexpr float GetDBLevelCalibrated(const float &value, const float &calibrationAmplitude) {
  float amp = fabsf(value);
  if (amp == 0.f || calibrationAmplitude <= 0.f) {
    return -std::numeric_limits<float>::infinity();
  }
  return 20.f * log10f(amp / calibrationAmplitude) + 94.f;
}

SoundLevelMeter::SoundLevelMeter(const SLMConfig &cfg)
    : mTimeWeighting(GetTimeWeightingTimeMs((TimeWeighting)cfg.tW))
    , mPeak(.0f)
    , mCalibratedLevel(DEFAULT_INMP441_CALIBRATION_VALUE) {
  reset(cfg);
}

void SoundLevelMeter::reset(const SLMConfig &cfg) {
  SLMConfig _c{cfg};
  switch (_c.fW) {
  case FrequencyWeighting::A:
    mFreqWeighting.reset(new Filtering::AWeighting());
    break;
  case FrequencyWeighting::C:
    mFreqWeighting.reset(new Filtering::CWeighting());
    break;
  case FrequencyWeighting::Z:
    mFreqWeighting.reset(nullptr);
    break;
  default:
    while (1);
  }
  mTimeWeighting.set_msec(GetTimeWeightingTimeMs(_c.tW));
}

MeterResults SoundLevelMeter::process(const float &sample) {
  MeterResults results;
  float fWeighted = sample;
  float tWeighted = .0f;
  if (mFreqWeighting) {
    mFreqWeighting->process(const_cast<float *>(&sample), &fWeighted, 1);
  }
  if (fWeighted > mPeak) {
    mPeak = fWeighted;
  }
  mTimeWeighting.process(&fWeighted, &tWeighted, 1);

  results.leq = GetDBLevelCalibrated(tWeighted, mCalibratedLevel);
  results.peak = GetDBLevelCalibrated(mPeak, mCalibratedLevel);
  results.spl = GetDBLevelCalibrated(sample, mCalibratedLevel);
  return results;
}

float SoundLevelMeter::calibrate(const float &sample) {
  mCalibratedLevel = fabsf(sample);
  if (mCalibratedLevel <= 0.f) {
    mCalibratedLevel = 1.f;
  }
  return GetDBLevelCalibrated(sample, mCalibratedLevel);
}

} // namespace slm
