#include "CWeighting.h"
#include "FilterCoeffs.h"
#include <algorithm>

namespace Filtering {

CWeighting::CWeighting()
    : mStage1 {
        {C_COEFFS[0].a0, C_COEFFS[0].a1, C_COEFFS[0].a2},
        {C_COEFFS[0].b0, C_COEFFS[0].b1, C_COEFFS[0].b2},
      },
      mStage2 {
        {C_COEFFS[1].a0, C_COEFFS[1].a1, C_COEFFS[1].a2},
        {C_COEFFS[1].b0, C_COEFFS[1].b1, C_COEFFS[1].b2},
      }
{}

const FilterStages<2> CWeighting::get_stages() {
  FilterStages<2> stages{&mStage1, &mStage2};
  return stages;
}

int CWeighting::process(float *inSamples, float *outSamples,
                        unsigned int nSamples) {
  auto stages = get_stages();
  auto validStagesCount =
      std::count_if(stages.begin(), stages.end(),
                    [](BiquadDFII *ptr) { return ptr != nullptr; });
  if (validStagesCount != stages.size()) {
    return -1;
  }

  for (size_t n = 0; n < nSamples; ++n) {
    float out = inSamples[n];
    for (auto stage : stages) {
      out = stage->process(out);
    }
    outSamples[n] = out;
  }

  return 0;
}

void CWeighting::clear_state() {
  auto stages = get_stages();
  for (auto stage : stages) {
    stage->clear_state();
  }
}

} // namespace Filtering
