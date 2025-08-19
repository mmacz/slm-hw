#include "SoundLevelMeter.h"
#include "AWeighting.h"
#include "CWeighting.h"
#include "FilterCoeffs.h"

#include <cstdint>
#include <cmath>

namespace slm {

static constexpr float INV_SQRT2 = 0.70710678118f;

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

SoundLevelMeter::SoundLevelMeter(const SLMConfig &cfg)
    : mTimeWeighting(GetTimeWeightingTimeMs((TimeWeighting)cfg.tW))
    , mLeqEnergySum(.0)
    , mLeqSamples(0)
    , mPeakAbs(.0f)
    , mSampleRef(DEFAULT_INMP441_CALIBRATION_VALUE)
    , mCalibratedLevel(DEFAULT_INMP441_CALIBRATION_VALUE * (1.0f / std::sqrt(2.0f)))
    , mCalGain(1.0f) {
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
  MeterResults r;
  float xw = sample;
  if (mFreqWeighting) {
    float tmp = sample;
    mFreqWeighting->process(&tmp, &xw);
  }

  const float a = fabsf(xw);
  if (a > mPeakAbs) mPeakAbs = a;

  float tRms = 0.0f;
  mTimeWeighting.process(&xw, &tRms);

  mLeqEnergySum += (double)(xw * xw);
  mLeqSamples   += 1;

  const float refA = mCalibratedLevel;
  const float refE = refA * refA;

  r.spl  = 10.0f * log10f( (tRms*tRms) / refE ) + 94.0f;
  r.leq  = 10.0f * log10f( (float)(mLeqEnergySum/(double)mLeqSamples) / refE ) + 94.0f;
  r.peak = 20.0f * log10f( mPeakAbs / refA ) + 94.0f;

  return r;
}

float SoundLevelMeter::calibrate(const float &sample) {
  if (!(mSampleRef > 0.f) || !std::isfinite(mSampleRef))
      mSampleRef = DEFAULT_INMP441_CALIBRATION_VALUE;

  float a = std::fabs(sample);
  if (!(a > 0.f) || !std::isfinite(a))
      return -std::numeric_limits<float>::infinity();

  float db_now = 20.0f * std::log10(a / mSampleRef) + 94.0f;

  float targetGain = mSampleRef / a;

  float alpha = .4f;
  if (!(alpha > 0.f) || alpha > 1.f) alpha = 0.05f;
  mCalGain = (1.0f - alpha) * mCalGain + alpha * targetGain;

  mCalibratedLevel = mSampleRef * INV_SQRT2;

  return db_now;
}

} // namespace slm
