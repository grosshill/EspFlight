set(SRC_FILES pid.c kalman.c mahony.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp
    REQUIRES utils
    REQUIRES sensors
)
