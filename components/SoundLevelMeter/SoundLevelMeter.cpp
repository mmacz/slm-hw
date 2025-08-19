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
    , mSampleRef(0.050118723f)
    , mRmsRef(mSampleRef * INV_SQRT2)
    , mCalGain(1.0f) {
  reset(cfg);
  while(true) {
    float dbCurrent = calibrate(mSampleRef, .1f);
    if (fabsf(dbCurrent - 94.f) <= .1f) {
      break;
    }
  }
  mRmsRef = 0.028f;
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
  float xw = sample * mCalGain;
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

  const float refA = mRmsRef;
  const float refE = refA * refA;

  r.spl  = 10.0f * log10f( (tRms*tRms) / refE ) + 94.0f;
  r.leq  = 10.0f * log10f( (float)(mLeqEnergySum/(double)mLeqSamples) / refE ) + 94.0f;
  r.peak = 20.0f * log10f( mPeakAbs / refA ) + 94.0f;

  return r;
}


float SoundLevelMeter::calibrate(const float &sample_now, float alpha) {
    if (!(mSampleRef > 0.f) || !std::isfinite(mSampleRef))
        mSampleRef = 0.050118723f;

    float a_raw = fabsf(sample_now);
    if (!(a_raw > 0.f) || !std::isfinite(a_raw))
        return -std::numeric_limits<float>::infinity();

    float a_eff = a_raw * mCalGain;

    float db_now = 20.0f * log10f(a_eff / mSampleRef) + 94.0f;

    float g_step = mSampleRef / a_eff;

    if (!(alpha > 0.f) || alpha > 1.f) alpha = 0.1f;

    float g_new_abs = mCalGain * g_step;
    if (g_new_abs > 100.f) g_new_abs = 100.f;
    if (g_new_abs < 0.01f) g_new_abs = 0.01f;

    mCalGain = (1.0f - alpha) * mCalGain + alpha * g_new_abs;

    mRmsRef = mSampleRef * 0.70710678f;

    return db_now;
}

} // namespace slm
