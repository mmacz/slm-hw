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
    , mRmsRef(mSampleRef)
    , mCalGain(0.707f) {
  reset(cfg);
  mCalibratedLevel = mRmsRef;
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
    float tmp = xw;
    mFreqWeighting->process(&tmp, &xw);
  }
  xw *= mCalGain;

  const float a = fabsf(xw);
  if (a > mPeakAbs) mPeakAbs = a;

  float tRms = 0.0f;
  mTimeWeighting.process(&xw, &tRms);

  mLeqEnergySum += (double)(xw * xw);
  mLeqSamples   += 1;

  const float refA = mRmsRef;
  const float refE = refA * refA;

  r.spl  = 10.0f * log10f( (tRms*tRms) / refE ) + 94.0f;
  r.leq  = 10.0f * log10f( (float)(mLeqEnergySum/(double)mLeqSamples) / refE ) + 94.0f;
  r.peak = 20.0f * log10f( mPeakAbs / refA ) + 94.0f;

  return r;
}


float SoundLevelMeter::calibrate(const float &sampleNow, float alpha) {
  if (!(mSampleRef > 0.f) || !std::isfinite(mSampleRef)) {
    mSampleRef = DEFAULT_INMP441_CALIBRATION_VALUE;
  }

  float sampleAbs = fabsf(sampleNow);
  if (!(sampleAbs > 0.f) || !std::isfinite(sampleAbs)) {
    return -std::numeric_limits<float>::infinity();
  }

  float sampleEff = sampleAbs * mCalGain;
  float dbNow = 20.0f * log10f(sampleEff / mSampleRef) + 94.0f;
  float gainStep = mSampleRef / sampleEff;

  if (!(alpha > 0.f) || alpha > 1.f) {
    alpha = 0.1f;
  }

  float gainNewAbs = mCalGain * gainStep;
  if (gainNewAbs > 100.f) {
    gainNewAbs = 100.f;
  }
  if (gainNewAbs < 0.01f) {
    gainNewAbs = 0.01f;
  }

  mCalGain = (1.0f - alpha) * mCalGain + alpha * gainNewAbs;

  return dbNow;
}

} // namespace slm
