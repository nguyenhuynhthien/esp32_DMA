#ifndef Q15_SIMD_HELPER_H
#define Q15_SIMD_HELPER_H

#include <stdint.h>
#include "../../include/Constants.h"

// A pair of signed Q15 values. On Xtensa DSP targets this maps to one
// 32-bit register and the native AE_* instructions are used.
typedef union {
    int16_t lane[2];
    int32_t packed;
} Q15x2;

static inline Q15x2 q15x2_make(int16_t high, int16_t low) {
    Q15x2 value;
    value.lane[0] = low;
    value.lane[1] = high;
    return value;
}

static inline Q15x2 q15x2_sub(Q15x2 a, Q15x2 b) {
    return q15x2_make((int16_t)(a.lane[1] - b.lane[1]),
                      (int16_t)(a.lane[0] - b.lane[0]));
}

static inline Q15x2 q15x2_add_sat_adc(const uint16_t* raw, int16_t mean) {
    int32_t low = ((int32_t)(raw[0] & Constant::ADC_RESOLUTION_MAX) - mean)
                  << Constant::Q15_SCALE_SHIFT;
    int32_t high = ((int32_t)(raw[1] & Constant::ADC_RESOLUTION_MAX) - mean)
                   << Constant::Q15_SCALE_SHIFT;
    low = (low > Constant::Q15_MAX) ? Constant::Q15_MAX :
          (low < Constant::Q15_MIN) ? Constant::Q15_MIN : low;
    high = (high > Constant::Q15_MAX) ? Constant::Q15_MAX :
           (high < Constant::Q15_MIN) ? Constant::Q15_MIN : high;
    return q15x2_make((int16_t)high, (int16_t)low);
}

static inline Q15x2 q15x2_add_sat(Q15x2 a, Q15x2 b) {
    int32_t low = (int32_t)a.lane[0] + b.lane[0];
    int32_t high = (int32_t)a.lane[1] + b.lane[1];
    low = (low > Constant::Q15_MAX) ? Constant::Q15_MAX :
          (low < Constant::Q15_MIN) ? Constant::Q15_MIN : low;
    high = (high > Constant::Q15_MAX) ? Constant::Q15_MAX :
           (high < Constant::Q15_MIN) ? Constant::Q15_MIN : high;
    return q15x2_make((int16_t)high, (int16_t)low);
}

#endif
