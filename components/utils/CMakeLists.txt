idf_component_register(SRCS "matrix_utils.cpp" "quaternion_utils.cpp" "debug_utils.c"
                       INCLUDE_DIRS "include" "../external"
                       REQUIRES external
                       REQUIRES esp-dsp
                    )