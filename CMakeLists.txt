cmake_minimum_required(VERSION 3.17.1)

set(CONF_FILE prj.conf)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(z_quad_rotor)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Includes
include_directories(app PRIVATE ~/junk/etl-master/include)

# Application sources
target_sources(app PRIVATE src/main.cpp)
target_sources(app PRIVATE src/marg_sensor.cpp)
target_sources(app PRIVATE src/pressure_sensor.cpp)