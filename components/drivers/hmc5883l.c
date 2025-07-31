#include "hmc5883l.h"
#include "i2c_manager.h"


esp_err_t hmc5883l_init(i2c_master_dev_handle_t dev_handle)
{   
    uint8_t data = HMC5883L_CONFIG_DEFAULT;
    esp_err_t ret = i2c_write(dev_handle, HMC5883L_CONFIG_A, &data, 1);
    if (ret != ESP_OK) return ret;

    data = HMC5883L_RANGE_DEFAULT;
    ret = i2c_write(dev_handle, HMC5883L_CONFIG_B, &data, 1);
    if (ret != ESP_OK) return ret;

    data = HMC5883L_CONTINUOUS_MODE;
    ret = i2c_write(dev_handle, HMC5883L_MODE, &data, 1);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}


esp_err_t hmc5883l_read_raw(i2c_master_dev_handle_t dev_handle, hmc5883l_pack_t* data_pack)
{
    uint8_t data[6];

    esp_err_t ret = i2c_read(dev_handle, HMC5883L_X_H, data, 6);
    if (ret != ESP_OK) return ret;

    int16_t dx, dy, dz;
	
    dx = (data[0] << 8) | data[1]; 
    data_pack->gx = (float) dx / HMC5883L_UNIT_DEFAULT;

    dy = (data[4] << 8) | data[5];
    data_pack->gy = (float) dy / HMC5883L_UNIT_DEFAULT;

    dz = (data[2] << 8) | data[3];
    data_pack->gz = (float) dz / HMC5883L_UNIT_DEFAULT;

    return ESP_OK;
}
