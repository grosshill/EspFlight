#include "fir_filter.h"
#include <malloc.h>


static __attribute__ ((aligned(16))) float signal[FIR_OUT_LEN];
static __attribute__ ((aligned(16))) float fir_coeffs[FIR_COEFFS_LEN];
static __attribute__ ((aligned(16))) float delay_line[FIR_COEFFS_LEN];

static __attribute__ ((aligned(16))) float window_input[FIR_SAMPLES];
const __attribute__ ((aligned(16))) float window[FIR_SAMPLES] = {1 / 256.f};
uint8_t window_ptr = 0;