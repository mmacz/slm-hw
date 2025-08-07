#include "BiquadDFII.h"

namespace Filtering {

BiquadDFII::BiquadDFII(FilterState a, FilterState b) : mA{a}, mB{b} {
  mD.fill(0);
}

float BiquadDFII::process(const float &inSample) {
  float output = (mB[0] * inSample) + mD[0];

  mD[0] = (mB[1] * inSample) - (mA[1] * output) + mD[1];
  mD[1] = (mB[2] * inSample) - (mA[2] * output);
  return output;
}

void BiquadDFII::clear_state() { mD.fill(0); }

} // namespace Filtering
