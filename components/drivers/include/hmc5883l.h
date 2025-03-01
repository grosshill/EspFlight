#ifndef HMC5883L_H
#define HMC5883L_H

#include "i2c_manager.h"

#define HMC5883L_DEVICE 0x0D //0x1E
#define HMC5883L_DEVICE_WRITE 0x3C
#define HMC5883L_DEVICE_READ 0x3D

#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01

#define HMC5883L_MODE 0x02

#define HMC5883L_X_H 0x03
#define HMC5883L_X_L 0x04
#define HMC5883L_Z_H 0x05
#define HMC5883L_Z_L 0x06
#define HMC5883L_Y_H 0x07
#define HMC5883L_Y_L 0x08

#define HMC5883L_STATE 0x09

#define HMC5883L_RECO_A 0x10
#define HMC5883L_RECO_B 0x11
#define HMC5883L_RECO_C 0x12

#define HMC5883L_CONFIG_DEFAULT 0x70
#define HMC5883L_RANGE_DEFAULT 0x20
#define HMC5883L_CONTINUOUS_MODE 0x00
#define HMC5883L_UNIT_DEFAULT 1090

#define HMC5883L_COLLABORATION_TIMEOUT (20 * 1000)

typedef struct{
    float gx;
    float gy;
    float gz;
} hmc5883l_pack_t;


esp_err_t hmc5883l_init(i2c_master_dev_handle_t dev_handle);

esp_err_t hmc5883l_read_raw(i2c_master_dev_handle_t dev_handle, hmc5883l_pack_t* data_pack);

esp_err_t hmc5883l_collaboration(i2c_master_dev_handle_t* dev_handle);

#endif