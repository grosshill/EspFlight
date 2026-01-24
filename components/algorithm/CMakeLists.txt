set(SRC_FILES pid.c kalman.c mahony.c fir_filter.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp
    REQUIRES utils
    REQUIRES sensors
    REQUIRES external
)
