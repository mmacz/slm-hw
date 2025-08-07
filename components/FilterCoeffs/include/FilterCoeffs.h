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
        { 1.00000000f, -0.14053608f, 0.00493760f, 0.25574113f, 0.51148225f, 0.25574113f },
        { 1.00000000f, -1.88490122f, 0.88642147f, 1.00000000f, -2.00015185f, 1.00015186f },
        { 1.00000000f, -1.99413888f, 0.99414747f, 1.00000000f, -1.99984815f, 0.99984816f },
    }
};

// C-weighting filter coefficients
constexpr std::size_t C_NUM_STAGES = 2;

constexpr std::array<BiquadCoeffs, C_NUM_STAGES> C_COEFFS = {
    {
        { 1.00000000f, -0.14053608f, 0.00493760f, 0.21700856f, 0.43401712f, 0.21700856f },
        { 1.00000000f, -1.99413888f, 0.99414747f, 1.00000000f, -2.00000000f, 1.00000000f },
    }
};

