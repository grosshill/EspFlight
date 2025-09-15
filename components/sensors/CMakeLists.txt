set(SRC_FILES 
    bmp280.c
    bmi270.c
    bmi270_config.c
    )

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES drivers
    REQUIRES algorithm
)