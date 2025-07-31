<<<<<<< HEAD
set(SRC_FILES pid.c kalman.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp
)
=======
set(SRC_FILES pid.c kalman.c quaternion.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp
)
>>>>>>> f59dd276765158714b13d0652314382d2513c103
