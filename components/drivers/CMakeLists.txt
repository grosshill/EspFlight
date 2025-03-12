set(SRC_FILES 
    i2c_manager.c
    mpu6050.c
    bmp180.c
    hmc5883l.c
    )

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver
    REQUIRES algorithm
)