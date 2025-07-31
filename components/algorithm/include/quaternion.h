#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float q0, q1, q2, q3;  
} Quaternion;

typedef struct {
    float roll, pitch, yaw;
} EulerAngles;

void quaternion_init(Quaternion* q);
void quaternion_update(Quaternion* q, float gx, float gy, float gz, float deltaT);
void quaternion_to_euler(Quaternion* q, EulerAngles* angles);
void print_euler(EulerAngles* angles);

#endif
