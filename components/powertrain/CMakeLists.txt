set(SRC_FILES
    bldc.c
    motor_pwm.c
    )

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES drivers
    REQUIRES algorithm
    REQUIRES utils
    REQUIRES esp_driver_ledc
)