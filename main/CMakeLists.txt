idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       REQUIRES algorithm
                       REQUIRES drivers
                       REQUIRES sensors
                       )
