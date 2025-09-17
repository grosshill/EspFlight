idf_component_register(SRCS "matrix_utils.cpp" "quaternion_utils.cpp" "debug_utils.c"
                       INCLUDE_DIRS "include" "../external"
                       REQUIRES external
                       REQUIRES esp-dsp
                       REQUIRES cxx newlib)
add_compile_definitions(
    EIGEN_NO_DEBUG
    EIGEN_MPL2_ONLY
    EIGEN_DONT_VECTORIZE
    EIGEN_DONT_ALIGN
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-maybe-uninitialized
    "-DEIGEN_NO_DEBUG"
    "-DEIGEN_MPL2_ONLY"
    "-DEIGEN_DONT_VECTORIZE" 
    "-std=c++11" 
)

set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
)