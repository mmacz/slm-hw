#pragma once

#include <array>
#include <cstddef>

constexpr float DEFAULT_INMP441_CALIBRATION_VALUE = 0.05010986f;

struct BiquadCoeffs {
    float a0, a1, a2;
    float b0, b1, b2;
};

// A-weighting filter coefficients
constexpr std::size_t A_NUM_STAGES = 3;

constexpr std::array<BiquadCoeffs, A_NUM_STAGES> A_COEFFS = {
    {
        { 1.00000000f, 0.45929809f, 0.05273868f, 0.42562639f, 0.85125278f, 0.42562639f },
        { 1.00000000f, -1.79605070f, 0.80094643f, 1.00000000f, -2.00017476f, 1.00017477f },
        { 1.00000000f, -1.98924339f, 0.98927232f, 1.00000000f, -1.99982524f, 0.99982526f },
    }
};

// C-weighting filter coefficients
constexpr std::size_t C_NUM_STAGES = 2;

constexpr std::array<BiquadCoeffs, C_NUM_STAGES> C_COEFFS = {
    {
        { 1.00000000f, 0.45929809f, 0.05273868f, 0.37866786f, 0.75733572f, 0.37866786f },
        { 1.00000000f, -1.98924339f, 0.98927232f, 1.00000000f, -2.00000000f, 1.00000000f },
    }
};

