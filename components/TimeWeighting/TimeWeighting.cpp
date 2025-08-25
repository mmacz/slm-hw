#include "TimeWeighting.h"
#include "FilterCoeffs.h"
#include <math.h>

namespace Filtering {

TimeWeighting::TimeWeighting(unsigned int mSec)
    : mStateE(0.f)
    , mSampleRate(SAMPLE_RATE) {
  mTau = static_cast<float>(mSec) / 1000.f;
  mdT = 1.0f / mSampleRate;
  mA = expf(-mdT / (mTau > 0.f ? mTau : mdT));
}

int TimeWeighting::process(float *inSamples, float *outSamples) {
  // square signal
  const float energy = *inSamples * *inSamples;
  mStateE = mA * mStateE + (1.0f - mA) * energy;
  *outSamples = sqrtf(mStateE);

  return 0;
}

void TimeWeighting::set_msec(unsigned int mSec) {
  mTau = (float)mSec / 1000.f;
  mA = expf(-mdT / (mTau > 0.f ? mTau : mdT));
}

void TimeWeighting::clear_state() { mStateE = 0; }

} // namespace Filtering
