set(SRC_FILES pid.c)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
)