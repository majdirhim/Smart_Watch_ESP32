idf_component_register(SRCS "SW_IMU_Driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "SW_I2c_Driver"
                    REQUIRES "lsm6dso_STdC_Driver")

