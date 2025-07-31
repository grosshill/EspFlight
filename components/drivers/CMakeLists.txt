set(SRC_FILES 
    i2c_manager.c
    )

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES driver
    REQUIRES algorithm
    REQUIRES utils
)