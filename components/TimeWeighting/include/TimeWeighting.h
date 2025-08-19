#pragma once
#include "FilterInterface.h"

namespace Filtering {
class TimeWeighting : FilterInterface {
public:
  TimeWeighting(unsigned int mSec);
  ~TimeWeighting() = default;

  int process(float *inSamples, float *outSamples) override;
  void clear_state() override;
  void set_msec(unsigned int mSec);

private:
  float mStateE;
  float mTau;
  float mA;
  float mSampleRate;
};
} // namespace Filtering
