#pragma once
#include "BiquadDFII.h"
#include "FilterInterface.h"

namespace Filtering {
class AWeighting : public FilterInterface {
public:
  AWeighting();
  ~AWeighting() = default;

  AWeighting(const AWeighting &) = delete;
  AWeighting(AWeighting &&) = delete;

  AWeighting &operator=(const AWeighting &) = delete;
  AWeighting &operator=(AWeighting &&) = delete;

  int process(float *inSamples, float *outSamples) override;
  void clear_state() override;

private:
  const FilterStages<3> get_stages();
  unsigned int mSampleRate;

  BiquadDFII mStage1;
  BiquadDFII mStage2;
  BiquadDFII mStage3;
};
} // namespace Filtering
