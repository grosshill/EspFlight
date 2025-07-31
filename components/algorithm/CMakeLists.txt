set(SRC_FILES pid.c kalman.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp
)