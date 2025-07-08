idf_component_register(SRCS "bmi270.cpp" "mpu6050.cpp" "bmp280.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES drivers
                    REQUIRES esp-dsp
                    REQUIRES algorithm
                    )