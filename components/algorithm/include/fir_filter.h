#pragma once
#include "esp_dsp.h"

#define FIR_COEFFS_LEN (32)
#define FIR_DECIM (2)
#define FIR_SAMPLES (256)
#define FIR_DELAY (FIR_COEFFS_LEN / FIR_DECIM)
#define FIR_OUT_LEN (FIR_SAMPLES + FIR_DELAY)

// static __attribute__ ((aligned(16))) float signal[FIR_OUT_LEN];
// static __attribute__ ((aligned(16))) float fir_coeffs[FIR_COEFFS_LEN];
// static __attribute__ ((aligned(16))) float delay_line[FIR_COEFFS_LEN];

fir_f32_t setup_fir_filter(void);

void window_filter(float new_sample, float* result);