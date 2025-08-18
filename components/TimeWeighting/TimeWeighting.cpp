#include "TimeWeighting.h"
#include "FilterCoeffs.h"
#include <math.h>

namespace Filtering {

TimeWeighting::TimeWeighting(unsigned int mSec)
    : mState(0.f)
    , mSampleRate(SAMPLE_RATE) {

  mTau = static_cast<float>(mSec) / 1000.f;
  mAlpha = mTau / (1.f / mSampleRate + mTau);
}

int TimeWeighting::process(float *inSamples, float *outSamples,
                           unsigned int nSamples) {
  // square signal
  for (unsigned int n = 0; n < nSamples; ++n) {
    inSamples[n] = powf(inSamples[n], 2);
  }

  outSamples[0] = mState * mAlpha + (1.0f - mAlpha) * inSamples[0];

  for (unsigned int n = 1; n < nSamples; ++n) {
    outSamples[n] = outSamples[n - 1] * mAlpha + (1.0f - mAlpha) * inSamples[n];
  }

  mState = outSamples[nSamples - 1];

  // square root
  for (unsigned int n = 0; n < nSamples; ++n) {
    outSamples[n] = sqrtf(outSamples[n]);
  }
  return 0;
}

void TimeWeighting::set_msec(unsigned int mSec) {
  mTau = static_cast<float>(mSec) / 1000.f;
  mAlpha = mTau / (1.f / mSampleRate + mTau);
}

void TimeWeighting::clear_state() { mState = 0; }

} // namespace Filtering
