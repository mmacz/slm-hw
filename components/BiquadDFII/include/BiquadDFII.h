#pragma once

#include <cstddef>
#include <array>

namespace Filtering {
class BiquadDFII;

using FilterState = std::array<float, 3>;

template <size_t N> using FilterStages = std::array<BiquadDFII *, N>;

class BiquadDFII {
public:
  BiquadDFII() = delete;
  BiquadDFII(FilterState a, FilterState b);
  ~BiquadDFII() = default;

  BiquadDFII(const BiquadDFII &) = delete;
  BiquadDFII(BiquadDFII &&) = delete;

  BiquadDFII &operator=(const BiquadDFII &) = delete;
  BiquadDFII &operator=(BiquadDFII &&) = delete;

  float process(const float &inSample);
  void clear_state();

private:
  FilterState mA;
  FilterState mB;
  FilterState mD;
};
} // namespace Filtering
