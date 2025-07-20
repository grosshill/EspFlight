#include "quaternion.h"
#include <stdio.h>
#include <math.h>

void quaternion_init(Quaternion* q) {
    q->q0 = 1.0;
    q->q1 = 0.0;
    q->q2 = 0.0;
    q->q3 = 0.0;
}

void quaternion_update(Quaternion* q, float gx, float gy, float gz, float deltaT) {
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;
    float q0 = q->q0;

q->q0 += 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
    q->q1 += 0.5 * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
    q->q2 += 0.5 * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
    q->q3 += 0.5 * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
}

void quaternion_to_euler(Quaternion* q, EulerAngles* angles) {
    angles->roll  = atan2(2.0f * (q->q0 * q->q1 + q->q2 * q->q3), 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2));
    angles->pitch = asin(2.0f * (q->q0 * q->q2 - q->q3 * q->q1));
    angles->yaw   = atan2(2.0f * (q->q0 * q->q3 + q->q1 * q->q2), 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3));
    angles->roll  = angles->roll * (180.0 / M_PI);
    angles->pitch = angles->pitch * (180.0 / M_PI);
    angles->yaw   = angles->yaw * (180.0 / M_PI);
}

void print_euler(EulerAngles* angles) {
    printf("Roll:  %.2f, Pitch: %.2f, Yaw: %.2f\n", angles->roll, angles->pitch, angles->yaw);
}

void setup() {
    Quaternion q;
    quaternion_init(&q);

    // 模拟传感器数据（角速度）
    float gx = 0.01; 
    float gy = 0.02; 
    float gz = 0.03;  
    float deltaT = 0.01;  

    quaternion_update(&q, gx, gy, gz, deltaT);

    EulerAngles angles;
    quaternion_to_euler(&q, &angles);
  
    print_euler(&angles);
}

void loop() {
    // 这里可以放置定时调用或传感器数据读取的部分
}
