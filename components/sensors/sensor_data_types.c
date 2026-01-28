#include "matrix_utils.h"
#include "sensor_data_types.h"
#include "stdlib.h"
#include <stdlib.h>
#ifdef __cplusplus
extern "C" {
#endif
state_t* state(void)
{
    state_t* s = malloc(sizeof(state_t));
    s->acc = vec3f_zeros();
    s->ang = vec3f_zeros();
    s->omg = vec3f_zeros();
    s->pos = vec3f_zeros();
    s->vel = vec3f_zeros();
    
    return s;
}

void _state(state_t* s)
{
    if(s)
    {
        vec3f_free(s->acc);
        vec3f_free(s->ang);
        vec3f_free(s->omg);
        vec3f_free(s->pos);
        vec3f_free(s->vel);
        free(s);
    }
}

#ifdef __cplusplus
}
#endif