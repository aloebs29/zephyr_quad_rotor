cmake_minimum_required(VERSION 3.10)

set(CONF_FILE prj.conf)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(z_quad_rotor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include directories
include_directories(app PRIVATE 
    lib/linalg
)

# Application sources
target_sources(app PRIVATE 
    src/fusion.cpp
    src/fxas21002.cpp
    src/fxos8700.cpp
    src/main.cpp
    src/pressure_sensor.cpp
)