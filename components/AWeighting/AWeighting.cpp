#include "AWeighting.h"
#include "FilterCoeffs.h"
#include <algorithm>

namespace Filtering {

AWeighting::AWeighting()
    : mStage1{{A_COEFFS[0].a0, A_COEFFS[0].a1, A_COEFFS[0].a2},
              {A_COEFFS[0].b0, A_COEFFS[0].b1, A_COEFFS[0].b2}},
      mStage2{{A_COEFFS[1].a0, A_COEFFS[1].a1, A_COEFFS[1].a2},
              {A_COEFFS[1].b0, A_COEFFS[1].b1, A_COEFFS[1].b2}},
      mStage3{{A_COEFFS[2].a0, A_COEFFS[2].a1, A_COEFFS[2].a2},
              {A_COEFFS[2].b0, A_COEFFS[2].b1, A_COEFFS[2].b2}} {}

const FilterStages<3> AWeighting::get_stages() {
  FilterStages<3> stages{&mStage1, &mStage2, &mStage3};
  return stages;
}

int AWeighting::process(float *inSamples, float *outSamples) {
  auto stages = get_stages();
  auto validStagesCount =
      std::count_if(stages.begin(), stages.end(),
                    [](BiquadDFII *ptr) { return ptr != nullptr; });
  if (validStagesCount != stages.size()) {
    return -1;
  }

  float out = *inSamples;
  for (auto stage : stages) {
    out = stage->process(out);
  }
  *outSamples = out;
  return 0;
}

void AWeighting::clear_state() {
  auto stages = get_stages();
  for (auto stage : stages) {
    stage->clear_state();
  }
}

} // namespace Filtering
