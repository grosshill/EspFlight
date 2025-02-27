#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


enum i2c_port_id {
    i2c_internal_port = 0,
    i2c_external_port
};

typedef struct {
    i2c_master_bus_config_t bus_cfg;  /* ESP-IDF i2c config */
    i2c_device_config_t dev_cfh;
    enum i2c_port_id port_id;             /* i2c port num */
} i2c_general_config_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} i2c_handle_t;

typedef struct {
    enum i2c_port_id port_id;
    uint8_t reg_addr;
    uint8_t* data_seq;
    uint32_t data_len;    
} i2c_write_handle_t;

typedef struct {
    enum i2c_port_id port_id;
    uint8_t reg_addr;

} i2c_read_handel_t;



#endif