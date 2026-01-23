#include "fir_filter.h"
#include <malloc.h>


static __attribute__ ((aligned(16))) float signal[FIR_OUT_LEN];
static __attribute__ ((aligned(16))) float fir_coeffs[FIR_COEFFS_LEN];
static __attribute__ ((aligned(16))) float delay_line[FIR_COEFFS_LEN];

static __attribute__ ((aligned(16))) float window_input[FIR_SAMPLES] = {0};
const __attribute__ ((aligned(16))) float window[FIR_SAMPLES] = {[0 ... FIR_SAMPLES - 1] = 1.0f / FIR_SAMPLES};
uint16_t window_ptr = 0;

void window_filter(float new_sample, float* result)
{   
    window_input[window_ptr] = new_sample;
    window_ptr = (window_ptr + 1) % FIR_SAMPLES;
    dsps_dotprod_f32_aes3(window_input, window, result, FIR_SAMPLES);
}