#pragma once
#include "BiquadDFII.h"
#include "FilterInterface.h"
#include <array>

namespace Filtering {
class CWeighting : public FilterInterface {
public:
  CWeighting();
  ~CWeighting() = default;

  CWeighting(const CWeighting &) = delete;
  CWeighting(CWeighting &&) = delete;

  CWeighting &operator=(const CWeighting &) = delete;
  CWeighting &operator=(CWeighting &&) = delete;

  int process(float *inSamples, float *outSamples) override;
  void clear_state() override;

private:
  const FilterStages<2> get_stages();
  unsigned int mSampleRate;

  BiquadDFII mStage1;
  BiquadDFII mStage2;
};
} // namespace Filtering
