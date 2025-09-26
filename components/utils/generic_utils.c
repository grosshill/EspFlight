#include "generic_utils.h"

float inv_sqrt(float x)
{
    float halfx = .5f * x;
    union {float f; int32_t i;} conv = {x};
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    conv.f *= 1.5f - (halfx * conv.f * conv.f);
    return conv.f;
}