idf_component_register(SRCS "DreamPilot.cpp" "modules/IMU.cpp"
        "modules/GPS/IGPSModule.cpp" "modules/GPS/NEO6M.cpp" "modules/GPS/NMEAParser.cpp"
        "modules/IMU/IIMUModule.cpp" "modules/IMU/MPU6050.cpp"
                    INCLUDE_DIRS "." "modules")
