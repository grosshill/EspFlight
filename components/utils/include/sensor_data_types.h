#pragma once

typedef struct {
    float ax;
    float ay;
    float az;
} acc_pack_t;

typedef struct {
    float gx;
    float gy;
    float gz;
} gyro_pack_t;

typedef struct {
    float mx;
    float my;
    float mz;
} mag_pack_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} atti_pack_t;

typedef struct {
    float temp;
} temp_pack_t;