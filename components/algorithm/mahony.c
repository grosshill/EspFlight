#include "mahony.h"
#include "esp_log.h"
#include "quaternion_utils.h"

void mahony_update(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t* params, float dt)
{
    /* Convert gyroscope degrees/sec to radians/sec */
    float gx = gyro.gx * deg2rad;
    float gy = gyro.gy * deg2rad;
    float gz = gyro.gz * deg2rad;
    float ax = acc.ax;
    float ay = acc.ay;
    float az = acc.az;
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float q0, q1, q2, q3;
    quatf_get(params->rotation, &q0, &q1, &q2, &q3);

    

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Estimated direction of gravity
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (params->Ki > 0.0f)
        {
            // integral error scaled by Ki
            params->inte_err_x += 2 * params->Ki * halfex * dt;
            params->inte_err_y += 2 * params->Ki * halfey * dt;
            params->inte_err_z += 2 * params->Ki * halfez * dt;
            gx += params->inte_err_x; // apply integral feedback
            gy += params->inte_err_y;
            gz += params->inte_err_z;
        } 
        else 
        {
            params->inte_err_x = 0.0f; // prevent integral windup
            params->inte_err_y = 0.0f;
            params->inte_err_z = 0.0f;
        }

        // Apply proportional feedback
        gx += 2 * params->Kp * halfex;
        gy += 2 * params->Kp * halfey;
        gz += 2 * params->Kp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    /*
        Auto normalization implemented by Eigen warpper
    */
    // norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    // q0 *= norm;
    // q1 *= norm;
    // q2 *= norm;
    // q3 *= norm;

    quatf_set(params->rotation, q0, q1, q2, q3);
}

void mahony_get_rad(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t *params, float dt)
{   
    float q0, q1, q2, q3;
    mahony_update(acc, gyro, params, dt);
    quatf_get(params->rotation, &q0, &q1, &q2, &q3);
    // ESP_LOGI("quat", "w: %f, x: %f, y: %f, z: %f", q0, q1, q2, q3);
    params->roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    params->pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    params->yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
    params->grax = 2.0f * (q1 * q3 - q0 * q2);
    params->gray = 2.0f * (q0 * q1 + q2 * q3);
    params->graz = 2.0f * (q0 * q0 - 0.5f + q3 * q3);
}

void mahony_get_deg(const acc_pack_t acc, const gyro_pack_t gyro, mahony_params_t *params, atti_pack_t* attitude, float dt)
{
    mahony_get_rad(acc, gyro, params, dt);
    attitude->roll = params->roll * rad2deg;
    attitude->pitch = params->pitch * rad2deg;
    attitude->yaw = params->yaw * rad2deg + 180.f;
    // ESP_LOGI("mahony", "roll %f, pitch %f, yaw %f", params->roll, params->pitch, params->yaw);
}