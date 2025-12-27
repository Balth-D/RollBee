file(GLOB_RECURSE sources   "main.cpp"
                            "board/*.cpp")

idf_component_register(SRCS "AppZigbee.cpp" "${sources}"
                       INCLUDE_DIRS ".")
