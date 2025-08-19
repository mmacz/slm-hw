#pragma once

#include <array>
#include <cstddef>

#include <cstdint>

constexpr float DEFAULT_INMP441_CALIBRATION_VALUE = 0.05010986f;
constexpr int32_t SAMPLE_RATE = 48000;

struct BiquadCoeffs {
    float a0, a1, a2;
    float b0, b1, b2;
};

// A-weighting filter coefficients
constexpr std::size_t A_NUM_STAGES = 3;

constexpr std::array<BiquadCoeffs, A_NUM_STAGES> A_COEFFS = {
    {
        { 1.00000000f, -0.22455846f, 0.01260663f, 0.23430179f, 0.46860358f, 0.23430179f },
        { 1.00000000f, -1.89387049f, 0.89515977f, 1.00000000f, -2.00000002f, 1.00000003f },
        { 1.00000000f, -1.99461446f, 0.99462171f, 1.00000000f, -1.99999998f, 0.99999997f },
    }
};

// C-weighting filter coefficients
constexpr std::size_t C_NUM_STAGES = 2;

constexpr std::array<BiquadCoeffs, C_NUM_STAGES> C_COEFFS = {
    {
        { 1.00000000f, -0.22455846f, 0.01260663f, 0.19788712f, 0.39577424f, 0.19788712f },
        { 1.00000000f, -1.99461446f, 0.99462171f, 1.00000000f, -2.00000000f, 1.00000000f },
    }
};

